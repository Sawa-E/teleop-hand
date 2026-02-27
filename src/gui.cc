#include "gui.h"

GLFWwindow *reset_and_open_window(const std::string window_name)
{
  const char* glsl_version = "#version 130";
  if (!glfwInit())
      exit(1);

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

  const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
  GLFWwindow *window = glfwCreateWindow(mode->width, mode->height, window_name.c_str(), NULL, NULL);
  if(window == nullptr){
    exit(1);
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1); // Enable vsync

  glfwSetKeyCallback(window, key_callback);

  bool err = gl3wInit() != 0;
  if (err)
  {
      fprintf(stderr, "Failed to initialize OpenGL loader!\n");
      exit(1);
  }

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();

  // Setup Platform/Renderer bindings
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  ImGuiIO& io = ImGui::GetIO();
  io.Fonts->AddFontFromFileTTF("../config/fonts/Roboto-Medium.ttf", 16.0f, NULL);
  return window;
}

void initialize_gui(GLFWwindow *window)
{
  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
}

void clean_up_window(GLFWwindow *window)
{
  // Cleanup
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    (void)window;
    glViewport(0, 0, width, height);
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    (void)scancode ;
    (void)mods ;
    if (key == GLFW_KEY_Q && action == GLFW_PRESS)
    {
      std::cout << "Q key!!" << std::endl ;
      glfwSetWindowShouldClose(window, GL_TRUE);
      std::cout << "Close !!" << std::endl ;
    }
}

void gui::show_widgets(const std::string &widget_name)
{
  IM_ASSERT(ImGui::GetCurrentContext() != NULL && "Missing dear imgui context. Refer to examples app!");
  ImGuiWindowFlags window_flags = 0;
  int width, height;

  glfwGetFramebufferSize(window_ptr_, &width, &height);
  ImGui::SetNextWindowPos(layout_.get_position(width, height, widget_name.c_str()));
  ImGui::SetNextWindowSize(layout_.get_size(width, height, widget_name.c_str()));
  if (!ImGui::Begin(widget_name.c_str(), &name_to_show_flag_and_widget[widget_name].first, window_flags))
  {ImGui::End();return;}
  // show registered widget
  name_to_show_flag_and_widget[widget_name].second(robot_system_ptr_);
  ImGui::End();
}

void gui::draw_windows()
{
  static ImVec4 clear_color(0.11f, 0.11f, 0.11f, 1.00f);
  glfwPollEvents();

  // Start dear imgui
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  // write gui widget
  for (auto &entry : name_to_show_flag_and_widget)
    if (entry.second.first)
      show_widgets(entry.first);

  // Rendering
  ImGui::Render();
  int display_w, display_h;
  glfwGetFramebufferSize(window_ptr_, &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);
  glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
  glClear(GL_COLOR_BUFFER_BIT);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  glfwSwapBuffers(window_ptr_);
}
