#include "framework.h"
#include "RenderingFramework.h"

#define MAX_LOADSTRING 100

#include "utils.hpp"
#include "mesh_model.hpp"
#include "camera_model.hpp"
#include "intersection.hpp"
#include "light_model.hpp"


HINSTANCE hInst;                               
WCHAR szTitle[MAX_LOADSTRING];                 
WCHAR szWindowClass[MAX_LOADSTRING];           
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);

//Ray Tracing definition
RayTracing main_model;

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPWSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);




    LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
    LoadStringW(hInstance, IDC_RENDERINGFRAMEWORK, szWindowClass, MAX_LOADSTRING);
    MyRegisterClass(hInstance);

    if (!InitInstance (hInstance, nCmdShow))
    {
        return FALSE;
    }



    HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_RENDERINGFRAMEWORK));

    MSG msg;

    while (GetMessage(&msg, nullptr, 0, 0))
    {
        if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
    }

    return (int) msg.wParam;
}




ATOM MyRegisterClass(HINSTANCE hInstance)
{
    WNDCLASSEXW wcex;

    wcex.cbSize = sizeof(WNDCLASSEX);

    wcex.style          = CS_HREDRAW | CS_VREDRAW;
    wcex.lpfnWndProc    = WndProc;
    wcex.cbClsExtra     = 0;
    wcex.cbWndExtra     = 0;
    wcex.hInstance      = hInstance;
    wcex.hIcon          = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_RENDERINGFRAMEWORK));
    wcex.hCursor        = LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground  = (HBRUSH)(COLOR_WINDOW+1);
    wcex.lpszMenuName   = MAKEINTRESOURCEW(IDC_RENDERINGFRAMEWORK);
    wcex.lpszClassName  = szWindowClass;
    wcex.hIconSm        = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

    return RegisterClassExW(&wcex);
}


BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
   hInst = hInstance; 
   HWND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
       CW_USEDEFAULT, 0, main_model.camera.width + 15, main_model.camera.height + 58, nullptr, nullptr, hInstance, nullptr);

   if (!hWnd)
   {
      return FALSE;
   }
   ShowWindow(hWnd, nCmdShow);
   UpdateWindow(hWnd);

   return TRUE;
}


LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
    case WM_COMMAND:
        {
            int wmId = LOWORD(wParam);
            switch (wmId)
            {
            case IDM_ABOUT:
                DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
                break;
            case IDM_EXIT:
                DestroyWindow(hWnd);
                break;
            default:
                return DefWindowProc(hWnd, message, wParam, lParam);
            }
        }
        break;
    case WM_LBUTTONDOWN: {
        HDC hdc = GetDC(hWnd);
        int x = LOWORD(lParam);
        int y = HIWORD(lParam);
        main_model.camera.MouseDown(x, y);
        ReleaseDC(hWnd, hdc);
        break;
    }
    case WM_RBUTTONDOWN: {
        HDC hdc = GetDC(hWnd);
        int x = LOWORD(lParam);
        int y = HIWORD(lParam);
        main_model.camera.MouseDown(x, y);
        ReleaseDC(hWnd, hdc);
        break;
    }
    case WM_LBUTTONUP: {
        main_model.camera.MouseUp();
        InvalidateRect(hWnd, NULL, true);
        break;
    }
    case WM_RBUTTONUP: {
        main_model.camera.MouseUp();
        InvalidateRect(hWnd, NULL, true);
        break;
    }
    case WM_MOUSEMOVE: {
        HDC hdc = GetDC(hWnd);
        int x = LOWORD(lParam);
        int y = HIWORD(lParam);
        main_model.camera.MouseMove(x, y);
        ReleaseDC(hWnd, hdc);
        break;
    }
    case WM_MOUSEWHEEL: {
        HDC hdc = GetDC(hWnd);
        main_model.camera.MouseWheel((short)HIWORD(wParam));
        InvalidateRect(hWnd, NULL, TRUE);
        ReleaseDC(hWnd, hdc);
        break;
    }
    case WM_KEYUP: {
        HDC hdc = GetDC(hWnd);
        Vector3d move_direction_camera;
        move_direction_camera << 0, 0, 0;
        bool flush = 0;
        switch (wParam)
        {
            case VK_LEFT: {
                move_direction_camera << -1, 0, 0;
                flush = 1;
                break;
            }
            case VK_RIGHT: {
                move_direction_camera << 1, 0, 0;
                flush = 1;
                break;
            }
            case VK_UP: {
                move_direction_camera << 0, 0, 1;
                flush = 1;
                break;
            }
            case VK_DOWN: {
                move_direction_camera << 0, 0, -1;
                flush = 1;
                break;
            }
        }
        if (flush)
        {
            main_model.camera.KeyUp(move_direction_camera);
            InvalidateRect(hWnd, NULL, TRUE);
        }
        ReleaseDC(hWnd, hdc);
        break;
    }
    case WM_PAINT:{
        PAINTSTRUCT ps;
        HDC hdc = BeginPaint(hWnd, &ps);
        main_model.Main();
        ShowPicture(main_model.results, main_model.camera.width, main_model.camera.height, hdc);
        EndPaint(hWnd, &ps);
        break;
    }
    case WM_DESTROY:
        PostQuitMessage(0);
        break;
    default:
        return DefWindowProc(hWnd, message, wParam, lParam);
    }
    return 0;
}

INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(lParam);
    switch (message)
    {
    case WM_INITDIALOG:
        return (INT_PTR)TRUE;

    case WM_COMMAND:
        if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
        {
            EndDialog(hDlg, LOWORD(wParam));
            return (INT_PTR)TRUE;
        }
        break;
    }
    return (INT_PTR)FALSE;
}
