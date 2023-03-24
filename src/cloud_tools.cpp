#include <simple_grasping/cloud_tools.h>

namespace simple_grasping
{

// http://en.wikipedia.org/wiki/HSL_and_HSV#Converting_to_RGB
// for points on a dark background you want somewhat lightened
// colors generally... back off the saturation (s)      
void hsv2rgb(const float h, const float s, const float v, float& r, float& g, float& b)
{
  float c = v * s;
  float hprime = h/60.0;
  float x = c * (1.0 - fabs(fmodf(hprime, 2.0f) - 1));

  r = g = b = 0;

  if (hprime < 1) {
    r = c; g = x;
  } else if (hprime < 2) {
    r = x; g = c;
  } else if (hprime < 3) {
    g = c; b = x;
  } else if (hprime < 4) {
    g = x; b = c;
  } else if (hprime < 5) {
    r = x; b = c;
  } else if (hprime < 6) {
    r = c; b = x;
  }

  float m = v - c;
  r += m; g+=m; b+=m;
}

void colorizeCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud, float hue)
{
  std::vector<pcl::PCLPointField> fields;
  pcl::getFields(cloud, fields);
  size_t rgb_field_index;
  for (rgb_field_index = 0; rgb_field_index < fields.size(); ++rgb_field_index)
  {
    if (fields[rgb_field_index].name == "rgb" ||
        fields[rgb_field_index].name == "rgba")
      break;
  }

  float r, g, b;
  hsv2rgb(hue, 0.8 /*saturation*/, 1.0 /*value*/, r, g, b);

  for (size_t j = 0; j < cloud.points.size(); ++j)
  {
    pcl::PointXYZRGB &p = cloud.points[j];
    unsigned char* pt_rgb = (unsigned char*) &p;
    pt_rgb += fields[rgb_field_index].offset;
    (*pt_rgb) = (unsigned char) (r * 255);
    (*(pt_rgb+1)) = (unsigned char) (g * 255);
    (*(pt_rgb+2)) = (unsigned char) (b * 255);
  }
}

double distancePointToPlane(const Eigen::Vector4f& point, const pcl::ModelCoefficients::Ptr plane)
{
  Eigen::Vector4f pp(point);
  pp[3] = 1.0;
  Eigen::Vector4f m(plane->values[0], plane->values[1], plane->values[2], plane->values[3]);
  return pp.dot(m);
}

}  // namespace simple_grasping
