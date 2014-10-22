#define _CRTDBG_MAP_ALLOC 
#include <stdlib.h>
#include <crtdbg.h>

#include "WinApp.h"
#include "WinForm.h"
#include "WinAccel.h"
#include <commctrl.h>
#include "../LibIO.h"

#pragma comment(lib,"comctl32.lib")
#pragma comment(linker,"/manifestdependency:\""		\
	"type='win32' "									\
	"name='Microsoft.Windows.Common-Controls' "		\
	"version='6.0.0.0' "							\
	"processorArchitecture='*' "					\
	"publicKeyToken='6595b64144ccf1df' "			\
	"language='*'\""								\
)

using namespace CoreLib::Basic;
using namespace CoreLib::IO;

namespace CoreLib
{
	namespace WinForm
	{
		const wchar_t * Application::ControlClassName = L"GxWinControl";
		const wchar_t * Application::FormClassName = L"GxWinForm";
		const wchar_t * Application::GLFormClassName = L"GxGLWinForm";
		int Application::globalUID = 0;
		BaseForm * Application::mainForm = 0;
		HINSTANCE Application::instance = 0;
		HINSTANCE Application::prevInstance = 0;
		Dictionary<HWND, Component*> * Application::components;
		Dictionary<int, Object*> * Application::objMap;
		String * Application::cmdLine;
		HWND Application::mainFormHandle = 0;
		void * Application::gdiToken = 0;
		int Application::cmdShow = 0;
		AccelTable * Application::accelTable = 0;
		LOGFONT Application::SysFont;
		bool Application::terminate = false;
		NotifyEvent * Application::onMainLoop = 0;

		int Application::GenerateGUID()
		{
			return globalUID++;
		}

		void Application::Init(HINSTANCE hIns, HINSTANCE hPrevIns, LPTSTR cmdLine, int show)
		{
			instance = hIns;
			components = new Dictionary<HWND, Component*>();
			Application::cmdLine = new String;
			Application::objMap = new Dictionary<int, Object*>();
			prevInstance = hPrevIns;
			*Application::cmdLine = cmdLine;
			cmdShow = show;
			GdiplusStartupInput input;
			GdiplusStartup(&gdiToken, &input, 0);

			RegisterControlClass();
			RegisterFormClass();
			RegisterGLFormClass();

			INITCOMMONCONTROLSEX picce;
			picce.dwSize = sizeof(INITCOMMONCONTROLSEX);
			picce.dwICC = ICC_ANIMATE_CLASS | ICC_BAR_CLASSES | ICC_COOL_CLASSES | ICC_DATE_CLASSES
				| ICC_HOTKEY_CLASS | ICC_INTERNET_CLASSES | ICC_LINK_CLASS | ICC_LISTVIEW_CLASSES
				| ICC_NATIVEFNTCTL_CLASS | ICC_PAGESCROLLER_CLASS | ICC_PROGRESS_CLASS | ICC_STANDARD_CLASSES
				| ICC_TAB_CLASSES | ICC_TREEVIEW_CLASSES | ICC_UPDOWN_CLASS | ICC_USEREX_CLASSES | 
				ICC_WIN95_CLASSES;
			InitCommonControlsEx(&picce);

		}

		void Application::SetMainLoopEventHandler(NotifyEvent * mainLoop)
		{
			onMainLoop = mainLoop;
		}

		void Application::Terminate()
		{
			PostQuitMessage(0);
			terminate = true;
		}

		LOGFONT Application::GetSysFont()
		{
			NONCLIENTMETRICS NonClientMetrics;
			NonClientMetrics.cbSize=sizeof(NONCLIENTMETRICS) - sizeof(NonClientMetrics.iPaddedBorderWidth);
			int rs = SystemParametersInfo(SPI_GETNONCLIENTMETRICS,sizeof(NONCLIENTMETRICS),&NonClientMetrics,0);
			int err = GetLastError();
			SysFont = NonClientMetrics.lfMessageFont;
			return SysFont;
		}

		void Application::RegisterObjectHandle(int handle, Object * obj)
		{
			objMap->Add(handle, obj);
		}

		Object * Application::GetObjectFromHandle(int handle)
		{
			Object * rs = 0;
			objMap->TryGetValue(handle, rs);
			return rs;
		}

		void Application::RegisterControlClass()
		{
			WNDCLASSEX wcex;

			wcex.cbSize = sizeof(WNDCLASSEX);

			wcex.style			= CS_HREDRAW | CS_VREDRAW | CS_DBLCLKS ;
			wcex.lpfnWndProc	= WndProc;
			wcex.cbClsExtra		= 0;
			wcex.cbWndExtra		= 0;
			wcex.hInstance		= Application::GetHandle();
			wcex.hIcon			= 0;
			wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
			wcex.hbrBackground	= (HBRUSH)(COLOR_BTNFACE+1);
			wcex.lpszMenuName	= 0;
			wcex.lpszClassName	= ControlClassName;
			wcex.hIconSm		= 0;

			RegisterClassEx(&wcex);
		}

		void Application::RegisterFormClass()
		{
			WNDCLASSEX wcex;

			wcex.cbSize = sizeof(WNDCLASSEX);

			wcex.style			= CS_HREDRAW | CS_VREDRAW | CS_OWNDC | CS_DBLCLKS ;
			wcex.lpfnWndProc	= WndProc;
			wcex.cbClsExtra		= 0;
			wcex.cbWndExtra		= 0;
			wcex.hInstance		= Application::GetHandle();
			wcex.hIcon			= 0;
			wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
			wcex.hbrBackground	= (HBRUSH)(COLOR_WINDOW+1);
			wcex.lpszMenuName	= 0;
			wcex.lpszClassName	= FormClassName;
			wcex.hIconSm		= 0;

			RegisterClassEx(&wcex);
		}

		void Application::RegisterGLFormClass()
		{
			WNDCLASSEX wcex;

			wcex.cbSize = sizeof(WNDCLASSEX);

			wcex.style			= CS_HREDRAW | CS_VREDRAW | CS_OWNDC | CS_DBLCLKS;
			wcex.lpfnWndProc	= WndProc;
			wcex.cbClsExtra		= 0;
			wcex.cbWndExtra		= 0;
			wcex.hInstance		= Application::GetHandle();
			wcex.hIcon			= 0;
			wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
			wcex.hbrBackground	= NULL;
			wcex.lpszMenuName	= 0;
			wcex.lpszClassName	= GLFormClassName;
			wcex.hIconSm		= 0;

			RegisterClassEx(&wcex);
		}

		HINSTANCE Application::GetHandle()
		{
			return instance;
		}


		void Application::Run(const BaseForm * MainForm, bool NonBlocking)
		{
			MSG msg;
			mainForm = const_cast<BaseForm*>(MainForm);
			mainFormHandle = MainForm->GetHandle();
			ShowWindow(mainFormHandle, SW_SHOW);
			UpdateWindow(mainFormHandle);
			while (!terminate)
			{
				int HasMsg = (NonBlocking?PeekMessage(&msg, NULL,0,0,TRUE):GetMessage(&msg, NULL, 0,0));
				if (HasMsg)
				{
					bool acc = false;
					HWND hwnd = GetActiveWindow();
					if (!accelTable || !TranslateAccelerator(hwnd, 
						accelTable->GetHandle(), &msg))
					{
						TranslateMessage(&msg);
						DispatchMessage(&msg);
					}
				}
				if (msg.message == WM_QUIT)
					terminate = true;
				if (onMainLoop)
				{
					onMainLoop->Invoke((Object*)MainForm, EventArgs());
				}
			}
		}

		BaseForm * Application::GetMainForm()
		{
			return mainForm;
		}

		void Application::SetAccel(AccelTable * acc)
		{
			accelTable = acc;
		}

		void Application::DoEvents()
		{
			MSG msg;
			while (PeekMessage(&msg, NULL, 0,0,PM_REMOVE))
			{
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
		}

		void Application::RegisterComponent(Component * com)
		{
			components->Add(com->GetHandle(), com);
		}

		void Application::UnRegisterComponent(Component * com)
		{
			components->Remove(com->GetHandle());
		}

		int Application::RegisterHandle(Object * obj)
		{
			int id = objMap->Count();
			objMap->Add(id, obj);
			return id;
		}

		Object * Application::GetObject(int handle)
		{
			Object * rs = 0;
			objMap->TryGetValue(handle, rs);
			return rs;
		}

		void Application::UnRegisterHandle(int handle)
		{
			objMap->Remove(handle);
		}

		HWND Application::GetMainFormHandle()
		{
			return mainFormHandle;
		}

		Component * Application::GetComponent(const HWND handle)
		{
			Component * rs = 0;
			components->TryGetValue(handle, rs);
			return rs;
		}

		void Application::Dispose()
		{
			{
				List<BaseForm *> forms;
				for (auto & c : *components)
				{
					if (c.Value)
					{
						BaseForm * ctrl = dynamic_cast<BaseForm*>(c.Value);
						if (ctrl)
						{
							forms.Add(ctrl);		
						}
						c.Value = 0;
					}
				}
				for (int i=0; i<forms.Count(); i++)
					delete forms[i];
				delete components;
				delete cmdLine;
				delete objMap;
				delete onMainLoop;
			}
			GdiplusShutdown(gdiToken);

			_CrtDumpMemoryLeaks();
		}

		String Application::GetCommandLine()
		{
			return *cmdLine;
		}

		String Application::GetExePath()
		{
			wchar_t fileName[500];
			::GetModuleFileName(NULL, fileName, 500);
			return Path::GetDirectoryName(fileName);
		}

		CommandLineParser::CommandLineParser(const String & cmdLine)
		{
			String profile = 
				L"String = {\"[^\"]*\"}\n"\
				L"Value = {\\d+(.\\d*)?}\n"\
				L"Token = {[^ \\t\\n\"\']+}\n"\
				L"#WhiteSpace = {\\s+}\n";
			MetaLexer lexer(profile);
			lexer.Parse(cmdLine, stream);
		}
	
		String CommandLineParser::GetFileName()
		{
			if (stream.Count() && stream.FirstNode()->Value.TypeID != 1)
				return stream.FirstNode()->Value.Str;
			else
				return L"";
		}

		bool CommandLineParser::OptionExists(const String & opt)
		{
			for (auto * node = stream.FirstNode(); node; node=node->GetNext())
			{
				if (node->Value.Str.Equals(opt, false))
				{
					return true;
				}
			}
			return false;
		}

		String CommandLineParser::GetOptionValue(const String & opt)
		{
			for (auto * node = stream.FirstNode(); node; node=node->GetNext())
			{
				if (node->Value.Str.Equals(opt, false))
				{
					node = node->GetNext();
					if (node)
						return node->Value.Str;
					else
						return L"";
				}
			}
			return L"";
		}
	
		String CommandLineParser::GetToken(int id)
		{
			auto node = stream.FirstNode();
			for (int i = 0; i<id; i++)
			{
				node = node->GetNext();
			}
			return node->Value.Str;
		}
	
		int CommandLineParser::GetTokenCount()
		{
			return stream.Count();
		}

		int SystemMetrics::GetScreenWidth()
		{
			return GetSystemMetrics(SM_CXSCREEN);
		}

		int SystemMetrics::GetScreenHeight()
		{
			return GetSystemMetrics(SM_CYSCREEN);
		}
	}
}