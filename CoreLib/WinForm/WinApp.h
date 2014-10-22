#ifndef GX_WIN_APP_H
#define GX_WIN_APP_H

#include "Windows.h"
#include "../Basic.h"
#include "WinAccel.h"
#include "WinMessage.h"
#include "../Regex/MetaLexer.h"
#include "Debug.h"

namespace CoreLib
{
	namespace WinForm
	{
		using namespace CoreLib::Text;

		class Component;
		class BaseForm;
		class AccelTable;

		class Application : public Object
		{
		private:
			static int globalUID;
			static HINSTANCE instance, prevInstance;
			static HWND mainFormHandle;
			static BaseForm * mainForm;
			static String *cmdLine;
			static Dictionary<HWND, Component* > *components;
			static Dictionary<int, Object *> *objMap;
			static int cmdShow;
			static void * gdiToken;
			static AccelTable * accelTable;
			static LOGFONT SysFont;
			static bool terminate;
			static NotifyEvent * onMainLoop;
			static void Application::RegisterControlClass();
			static void Application::RegisterFormClass();
			static void Application::RegisterGLFormClass();
		public:
			static const wchar_t * ControlClassName;
			static const wchar_t * FormClassName;
			static const wchar_t * GLFormClassName;
			static void Init(HINSTANCE hIns, HINSTANCE hPrevIns, LPTSTR cmdLine, int show = 0);
			static void Dispose();
			static void RegisterComponent(Component * Ctrl);
			static void UnRegisterComponent(Component *  Ctrl);
			static Component * GetComponent(const HWND handle); 
			static HINSTANCE GetHandle();
			static void SetMainLoopEventHandler(NotifyEvent * mainLoop);
			static void Run(const BaseForm * MainForm, bool NonBlocking = false);
			static void Terminate();
			static HWND GetMainFormHandle();
			static BaseForm * GetMainForm();
			static void DoEvents();
			static int RegisterHandle(Object * obj);
			static void UnRegisterHandle(int _handle);
			static Object * GetObject(int _handle);
			static String GetCommandLine();
			static String GetExePath();
			static void SetAccel(AccelTable * acc);
			static void RegisterObjectHandle(int handle, Object * obj);
			static Object * GetObjectFromHandle(int handle);
			static LOGFONT GetSysFont();
			static int GenerateGUID();
		};

		class CommandLineParser : public Object
		{
		private:
			LexStream stream;
		public:
			CommandLineParser(const String & cmdLine);
			String GetFileName();
			bool OptionExists(const String & opt);
			String GetOptionValue(const String & opt);
			String GetToken(int id);
			int GetTokenCount();
		};

		class SystemMetrics
		{
		public:
			static int GetScreenWidth();
			static int GetScreenHeight();
		};
	}
}

#endif