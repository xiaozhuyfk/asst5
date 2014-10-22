#ifndef GX_WINFORM_H
#define GX_WINFORM_H

#include "WinCtrls.h"
#include "WinMenu.h"
#include "WinAccel.h"

namespace CoreLib
{
	namespace WinForm
	{
		class Button;

		enum FormBorder
		{
			fbNone,
			fbSingle,
			fbSizeable,
			fbFixedDialog,
			fbToolWindow,
			fbFixedToolWindow
		};

		class BaseForm abstract: public BaseControl 
		{
		public:
			enum DialogResult
			{
				OK, Cancel, Yes, No, Retry, Ignore
			};
		protected:
			DialogResult dlgResult;
			bool closed, firstShown;
			BaseForm * dlgOwner;
			Button * defButton, * cancelButton;
			bool ProcessMessage(WinMessage msg);
			virtual void Create()=0;
		private:
			MainMenu * menu;
			RefPtr<AccelTable> accelTable;
		public:
			BaseForm();
			virtual ~BaseForm();
		public:
			void		Show();
			DialogResult ShowModal(BaseForm * owner);
			void		Close();
			void		SetDialogResult(DialogResult result);
			void		SetText(String text);
			String	GetText();
			void		SetBorder(FormBorder border);
			FormBorder	GetBorder();
			bool		GetShowInTaskBar();
			void		SetShowInTaskBar(bool val);
			bool		GetControlBox();
			void		SetControlBox(bool val);
			bool		GetTopMost();
			void		SetTopMost(bool val);
			bool		GetMinimizeBox();
			void		SetMinimizeBox(bool val);
			bool		GetMaximizeBox();
			void		SetMaximizeBox(bool val);
			void		SetClientWidth(int w);
			void		SetClientHeight(int h);
			int			GetClientWidth();
			int			GetClientHeight();
			void		SetMainMenu(MainMenu * _menu);
			void		SetDefaultButton(Button * btn);
			Button *	GetDefaultButton();
			void		SetCancelButton(Button * btn);
			Button *	GetCancelButton();
			int			MessageBox(const String & msg, const String & title, unsigned int style);
		public:
			NotifyEvent OnLoad;
			NotifyEvent OnShow;
			WindowCloseEvent OnClose;
			void RegisterAccel(Accelerator acc, MenuItem * item);
			void UpdateAccel();
		};

		class Form : public BaseForm
		{
		protected:
			virtual void Create();
		public:
			Form();
			~Form();
		};
	}
}
#endif