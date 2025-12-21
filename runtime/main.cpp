#ifdef __EMSCRIPTEN__
  #include <emscripten/emscripten.h>
#endif

#include "core/Engine.h"
#include "core/Input.h"

#ifndef __EMSCRIPTEN__
  // Minimal native windowing: Xlib + GLX (no SDL/GLFW).
  #include <X11/Xlib.h>
  #include <X11/keysym.h>
  #include <GL/gl.h>
  #include <GL/glx.h>
  #include <cstdio>

  static Input::Key mapKeySym(KeySym sym) {
    switch (sym) {
      case XK_w: case XK_W: return Input::KEY_W;
      case XK_a: case XK_A: return Input::KEY_A;
      case XK_s: case XK_S: return Input::KEY_S;
      case XK_d: case XK_D: return Input::KEY_D;
      case XK_Shift_L: case XK_Shift_R: return Input::KEY_SHIFT;
      case XK_space: return Input::KEY_SPACE;
      case XK_Control_L: case XK_Control_R: return Input::KEY_CTRL;
      default: return Input::KEY_COUNT;
    }
  }
#endif

static void tick(void*) {
  Engine::instance().stepOnce();
}

int main() {
  Engine::instance().init();

#ifdef __EMSCRIPTEN__
  emscripten_set_main_loop_arg(tick, nullptr, 0, true);
  return 0; // browser owns the loop
#else
  Display* dpy = XOpenDisplay(nullptr);
  if (!dpy) {
    std::fprintf(stderr, "Failed to open X display\n");
    return 1;
  }

  const int screen = DefaultScreen(dpy);
  static int visualAttribs[] = {
      GLX_RGBA,
      GLX_DOUBLEBUFFER,
      GLX_DEPTH_SIZE, 24,
      GLX_STENCIL_SIZE, 8,
      None
  };
  XVisualInfo* vi = glXChooseVisual(dpy, screen, visualAttribs);
  if (!vi) {
    std::fprintf(stderr, "No appropriate GLX visual found\n");
    return 1;
  }

  Colormap cmap = XCreateColormap(dpy, RootWindow(dpy, screen), vi->visual, AllocNone);
  XSetWindowAttributes swa{};
  swa.colormap = cmap;
  swa.event_mask = ExposureMask | KeyPressMask | KeyReleaseMask |
                   ButtonPressMask | ButtonReleaseMask | PointerMotionMask |
                   StructureNotifyMask;

  const int width = 1280;
  const int height = 720;
  Window win = XCreateWindow(dpy, RootWindow(dpy, screen),
                             0, 0, width, height, 0,
                             vi->depth, InputOutput, vi->visual,
                             CWColormap | CWEventMask, &swa);
  XStoreName(dpy, win, "FickerEngine (X11)");
  Atom wmDelete = XInternAtom(dpy, "WM_DELETE_WINDOW", False);
  XSetWMProtocols(dpy, win, &wmDelete, 1);
  XMapWindow(dpy, win);

  GLXContext glc = glXCreateContext(dpy, vi, 0, GL_TRUE);
  glXMakeCurrent(dpy, win, glc);

  // Basic GL state.
  glViewport(0, 0, width, height);
  glEnable(GL_DEPTH_TEST);

  // Input state for mouse delta.
  int lastMouseX = 0;
  int lastMouseY = 0;
  bool hasLastMouse = false;

  bool running = true;
  while (running) {
    while (XPending(dpy)) {
      XEvent ev;
      XNextEvent(dpy, &ev);
      switch (ev.type) {
        case ClientMessage:
          if (static_cast<Atom>(ev.xclient.data.l[0]) == wmDelete) {
            running = false;
          }
          break;
        case ConfigureNotify:
          glViewport(0, 0, ev.xconfigure.width, ev.xconfigure.height);
          break;
        case KeyPress:
        case KeyRelease: {
          const bool down = (ev.type == KeyPress);
          KeySym sym = XLookupKeysym(&ev.xkey, 0);
          Input::Key k = mapKeySym(sym);
          if (k != Input::KEY_COUNT) Input::setKeyDown(k, down);
          break;
        }
        case ButtonPress:
        case ButtonRelease: {
          const bool down = (ev.type == ButtonPress);
          if (ev.xbutton.button == Button1) Input::setMouseButtonDown(Input::MOUSE_LEFT, down);
          if (ev.xbutton.button == Button2) Input::setMouseButtonDown(Input::MOUSE_MIDDLE, down);
          if (ev.xbutton.button == Button3) Input::setMouseButtonDown(Input::MOUSE_RIGHT, down);
          break;
        }
        case MotionNotify: {
          int mx = ev.xmotion.x;
          int my = ev.xmotion.y;
          if (!hasLastMouse) {
            lastMouseX = mx;
            lastMouseY = my;
            hasLastMouse = true;
          } else {
            Input::addMouseDelta(static_cast<float>(mx - lastMouseX),
                                 static_cast<float>(my - lastMouseY));
            lastMouseX = mx;
            lastMouseY = my;
          }
          break;
        }
      }
    }

    Engine::instance().stepOnce();
    glXSwapBuffers(dpy, win);
  }

  glXMakeCurrent(dpy, None, nullptr);
  glXDestroyContext(dpy, glc);
  XDestroyWindow(dpy, win);
  XCloseDisplay(dpy);

  Engine::instance().shutdown();
  return 0;
#endif
}
