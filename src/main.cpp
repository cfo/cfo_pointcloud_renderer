#include <pangolin/pangolin.h>
#include <sophus/se3.h>
#include <Eigen/Core>
#include <Eigen/Geometry> // Quaternion
#include <fstream>
#include <iostream> // ofstream
#include <vector>
#include <list>
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <opencv2/opencv.hpp>
#include <vikit/math_utils.h>

using namespace pangolin;
using namespace Eigen;
using namespace Sophus;

typedef Eigen::Matrix<unsigned char, 3, 1> Vector3u;

bool checkForGlError()
{
  GLenum error_code = glGetError();
  if (error_code==GL_NO_ERROR)
  {
    return true;
  }
  fprintf( stderr, "ERROR: %s\n", (char*)gluErrorString(error_code) );
  return false;
}

void glDrawPointcloud(
    const vector<Vector3f>& point_vec,
    const vector<Vector3u>& color_vec,
    double point_size)
{
  assert(point_vec.size() == color_vec.size());
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);

  glPointSize(point_size);
  glVertexPointer(3, GL_FLOAT, 0, &(point_vec[0][0]));
  glColorPointer(3, GL_UNSIGNED_BYTE, 0, &(color_vec[0][0]));
  glDrawArrays(GL_POINTS, 0, point_vec.size());

  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
  assert(checkForGlError());
}

bool loadPointsFromFile(
    const std::string& file_name,
    const size_t max_num_points,
    std::vector<Vector3f>& point_vec,
    std::vector<Vector3u>& color_vec)
{
  point_vec.clear();
  color_vec.clear();
  point_vec.reserve(max_num_points);
  color_vec.reserve(max_num_points);
  std::ifstream fs(file_name.c_str());
  if(!fs.is_open())
  {
    std::cout << "could not open points file " << file_name << std::endl;
    return false;
  }

  size_t i = 0;
  for(; i<max_num_points; ++i)
  {
    int u, v;
    float x, y, z;
    int r,g,b;
    fs >> u >> v >> x >> y >> z >> r >> g >> b;
    const Vector3f xyz_faro(x,y,z);
    const Vector3u rgb(r,g,b);
    point_vec.push_back(xyz_faro);
    color_vec.push_back(rgb);
    if(!fs.good())
    {
      std::cout << "fs is not good anymore" << std::endl;
      break;
    }
    if(fs.eof())
    {
      std::cout << "reached end of file" << std::endl;
      break;
    }
  }
  std::cout << "loaded " << i  << " points with color." << std::endl;
  return true;
}

bool loadTrajectoryFromFile(
    const std::string& file_name,
    const SE3& T_faro_optitrack,
    std::list<std::pair<double, SE3> >& T_world_robot_vec)
{
  T_world_robot_vec.clear();
  std::ifstream fs(file_name.c_str());
  if(!fs.is_open())
  {
    std::cout << "could not open trajectory file " << file_name << std::endl;
    return false;
  }

  while(fs.good() && !fs.eof())
  {
    double timestamp;
    double tx, ty, tz;
    double qx, qy, qz, qw;
    fs >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    SE3 T_optitrack_mav(Quaterniond(qw, qx, qy, qz), Vector3d(tx, ty, tz));
    SE3 T_faro_mav(T_faro_optitrack*T_optitrack_mav);

    // check if we have already a reference frame very close!
    bool found_close = false;
    for(auto T_world_robot : T_world_robot_vec)
    {
      if( (T_faro_mav.inverse()*T_world_robot.second).translation().norm() < 0.1 )
      {
        found_close = true;
        break;
      }
    }
    if(!found_close)
    {
      T_world_robot_vec.push_back(std::make_pair(timestamp, T_faro_mav));
    }
  }
  return true;
}

int main(int argc, char** argv)
{  
  // define sensor
  int sensor_width = 752;
  int sensor_height = 480;
  float fx = 418;
  float fy = 418;
  float cx = 397;
  float cy = 246;
  float clip_near = 0.1;
  float clip_far = 1000.0;

  // load points
  std::string points_file_name("data/data.xyz");
  std::vector<Vector3f> point_vec;
  std::vector<Vector3u> color_vec;


  //SE3 T_optitrack_faro(vk::rpy2dcm(Vector3d(0,0,0.78)), Vector3d(0.14, 0.25, 0.8));
  Quaterniond q_faro_optitrack(0.858, -0.011,  0.002, -0.514);
  Vector3d t_faro_optitrack(-0.1, -0.255, -1.036);
  SE3 T_faro_optitrack(q_faro_optitrack, t_faro_optitrack);
  if(!loadPointsFromFile(points_file_name, 18000000, point_vec, color_vec))
    return 1;

  // load trajectory
  std::string trajectory_file_name("data/trajectory.txt");
  std::list<std::pair<double, SE3> > T_world_robot_vec;
  if(!loadTrajectoryFromFile(trajectory_file_name, T_faro_optitrack, T_world_robot_vec))
    return 1;

  // set transformations
  Sophus::SE3 T_world_robot = T_world_robot_vec.front().second;
  Sophus::SE3 T_world_cam = T_world_robot; // opengl z is backwards
  Eigen::Matrix4d T_cam_world(T_world_cam.inverse().matrix());

  // Create OpenGL window in single line thanks to GLUT
  pangolin::CreateWindowAndBind("Main",sensor_width,sensor_height);
  
  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);
  
  // Issue specific OpenGl we might need
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Define Camera Projection Matrix and Pose. OpenGL uses Right-Up-Back projection
  // model and the image origin is in the lower left corner.
  pangolin::OpenGlMatrix Kgl =
      pangolin::ProjectionMatrixRUB_BottomLeft(
        sensor_width, sensor_height, fx, fy, cx, cy, clip_near, clip_far);
  pangolin::OpenGlMatrix Tgl(T_cam_world);
  pangolin::OpenGlRenderState cam_state(Kgl, Tgl);

  // Add named OpenGL viewport to window and provide 3D Handler
  View& d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0, 1.0, -static_cast<float>(sensor_width)/sensor_height);
    //.SetHandler(new Handler3D(s_cam));

  std::ofstream ofs("data/renderings.txt");
  assert(ofs.is_open());
  ofs.precision(10);

  const Viewport& viewport = d_cam.v;

  int i=0;
  bool quit = false;

  cv::Mat img(sensor_height, sensor_width, CV_8UC4), img_flipped;
  cv::Mat depth(sensor_height, sensor_width, CV_32F), depth_flipped;
  cv::Mat mask_flipped(sensor_height, sensor_width, CV_8UC1);

  while(!quit && !T_world_robot_vec.empty())
  {
    T_world_robot_vec.pop_front();
    if(i++%50 != 0)
      ; //continue;

    //--------------------------------------------------------------------------
    // INITIALIZING

    // Clear entire screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Activate efficiently by object
    double timestamp = T_world_robot_vec.front().first;
    T_world_robot = T_world_robot_vec.front().second;
    T_world_cam = T_world_robot;
    T_cam_world = T_world_cam.inverse().matrix();
    Tgl = pangolin::OpenGlMatrix(T_cam_world);

    // Set projectiview view
    glViewport(viewport.l, viewport.b, viewport.w, viewport.h);
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixd(Kgl.m);
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixd(Tgl.m);

    //--------------------------------------------------------------------------
    // DRAWING

    glDrawPointcloud(point_vec, color_vec, 0.5);
    //glDrawAxis(1.0);


    //--------------------------------------------------------------------------
    // SAVING

    std::string prefix("data/frame_"+std::to_string(i));

    // load image into cv::Mat
    glPixelStorei(GL_PACK_ALIGNMENT, (img.step & 3) ? 1 : 4); // use fast 4-byte alignment (default anyway) if possible
    glPixelStorei(GL_PACK_ROW_LENGTH, img.step/img.elemSize()); // set length of one complete row in destination data (doesn't need to equal img.cols)
    glReadPixels(viewport.l, viewport.b, viewport.w, viewport.h, GL_BGRA, GL_UNSIGNED_BYTE, img.data);
    cv::flip(img, img_flipped, 0);

    // load depth into cv::Mat
    glPixelStorei(GL_PACK_ALIGNMENT, (depth.step & 3) ? 1 : 4); // use fast 4-byte alignment (default anyway) if possible
    glPixelStorei(GL_PACK_ROW_LENGTH, depth.step/depth.elemSize()); // set length of one complete row in destination data (doesn't need to equal img.cols)
    glReadPixels(viewport.l, viewport.b, viewport.w, viewport.h, GL_DEPTH_COMPONENT, GL_FLOAT, depth.data);
    cv::flip(depth, depth_flipped, 0);

    // compute mask and true depth
    // http://web.archive.org/web/20130416194336/http://olivers.posterous.com/linear-depth-in-glsl-for-real
    float* depth_ptr = reinterpret_cast<float*>(depth_flipped.data);
    uint8_t* mask_ptr = mask_flipped.data;
    for(size_t i=0, ie=mask_flipped.cols*mask_flipped.rows; i<ie; ++i, ++depth_ptr, ++mask_ptr) {
      *mask_ptr = (*depth_ptr == 1.0) ? 0 : 255;

      float z_b = *depth_ptr; //0.5*(*depth_ptr) + 0.5; // z_b in [0,1]
      *depth_ptr = 2.0*clip_far*clip_near / (clip_far + clip_near - (clip_far - clip_near)*(2.0*z_b-1.0)); // true depth
    }

    // save images, convert float image as large uint8_t image
    cv::Mat depth_flipped_as_4uint(
          depth_flipped.rows, depth_flipped.cols*4, CV_8U, depth_flipped.data);
    cv::imwrite(prefix+".png", img_flipped);
    cv::imwrite(prefix+"_depth.png", depth_flipped_as_4uint);
    cv::imwrite(prefix+"_mask.png", mask_flipped);

    //--------------------------------------------------------------------------
    // FINISHING

    // reset viewport
    DisplayBase().Activate();
    Viewport::DisableScissor(); // glScissor() defines a screen space rectangle beyond which nothing is drawn

    // finish glut
    glutSwapBuffers();          // swap the buffers of the current window if double buffered.
    glutMainLoopEvent();        // dispatch all pending events
    quit = !glutGetWindow();    // dispatch all pending events

    // trace pose
    Matrix3d R_robot_cam; R_robot_cam << 1, 0, 0,  0, -1, 0,  0, 0, -1;
    SE3 T_robot_cam(R_robot_cam, Vector3d::Zero());
    Vector3d t = (T_world_robot*T_robot_cam).translation();
    Quaterniond q = (T_world_robot*T_robot_cam).unit_quaternion();
    ofs << timestamp << " "
        << "frame_"+std::to_string(i) << " "
        << t.x() << " " << t.y() << " " << t.z() << " "
        << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }

  return 0;
}
