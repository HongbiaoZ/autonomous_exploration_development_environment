// Ground-truth terrain lookup for vehicleSimulator.
//
// Reads the heightmap.bin written by dem_to_gazebo.py (marsrover-ros2,
// mars_ws/src/autonomy/autonomy/maps). File layout:
//
//   DEMGRID 1 <cols> <rows> <x_min> <y_max> <resolution>\n
//   rows * cols little-endian float32 heights (map frame, metres)
//
// Row 0 is the northern edge (y = y_max), column 0 the western edge
// (x = x_min). Each grid cell is split into two triangles along the
// south-west / north-east diagonal, exactly like terrain.obj, so height()
// returns the same surface the simulated lidar sees in Gazebo.

#ifndef VEHICLE_SIMULATOR_TERRAIN_MAP_H
#define VEHICLE_SIMULATOR_TERRAIN_MAP_H

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class TerrainMap
{
public:
  // Returns false and fills error if the file is missing or malformed.
  bool load(const std::string& path, std::string& error)
  {
    std::ifstream file(path, std::ios::binary);
    if (!file)
    {
      error = "cannot open " + path;
      return false;
    }

    std::string header;
    std::getline(file, header);
    std::istringstream fields(header);
    std::string magic;
    int version = 0;
    fields >> magic >> version >> cols_ >> rows_ >> xMin_ >> yMax_ >> res_;
    if (fields.fail() || magic != "DEMGRID" || version != 1 || cols_ < 2 || rows_ < 2 || res_ <= 0)
    {
      error = "bad header in " + path + ": '" + header + "'";
      return false;
    }

    heights_.resize(static_cast<size_t>(cols_) * rows_);
    file.read(reinterpret_cast<char*>(heights_.data()), heights_.size() * sizeof(float));
    if (file.gcount() != static_cast<std::streamsize>(heights_.size() * sizeof(float)))
    {
      error = "truncated height data in " + path;
      heights_.clear();
      return false;
    }
    return true;
  }

  bool loaded() const { return !heights_.empty(); }
  int cols() const { return cols_; }
  int rows() const { return rows_; }
  double resolution() const { return res_; }

  // Ground height at (x, y) in the map frame. False if outside the map.
  bool height(double x, double y, double& z) const
  {
    if (!loaded())
      return false;
    const double col = (x - xMin_) / res_;
    const double row = (yMax_ - y) / res_;
    if (col < 0 || row < 0 || col > cols_ - 1 || row > rows_ - 1)
      return false;

    int j = static_cast<int>(col);
    int i = static_cast<int>(row);
    if (j > cols_ - 2) j = cols_ - 2;
    if (i > rows_ - 2) i = rows_ - 2;
    const double u = col - j;  // fraction east
    const double v = row - i;  // fraction south

    const double za = at(i, j);          // north-west
    const double zb = at(i + 1, j);      // south-west
    const double zc = at(i, j + 1);      // north-east
    const double zd = at(i + 1, j + 1);  // south-east
    if (u + v <= 1.0)
      z = za + u * (zc - za) + v * (zb - za);
    else
      z = zd + (1.0 - u) * (zb - zd) + (1.0 - v) * (zc - zd);
    return true;
  }

  // Ground slope around (x, y), averaged over +/- radius (roughly half the
  // vehicle footprint) with central differences. dzdx is the rise per metre
  // east, dzdy per metre north. False if any sample is outside the map.
  bool slope(double x, double y, double radius, double& dzdx, double& dzdy) const
  {
    double ze, zw, zn, zs;
    if (!height(x + radius, y, ze) || !height(x - radius, y, zw) ||
        !height(x, y + radius, zn) || !height(x, y - radius, zs))
      return false;
    dzdx = (ze - zw) / (2.0 * radius);
    dzdy = (zn - zs) / (2.0 * radius);
    return true;
  }

private:
  double at(int i, int j) const { return heights_[static_cast<size_t>(i) * cols_ + j]; }

  int cols_ = 0;
  int rows_ = 0;
  double xMin_ = 0;
  double yMax_ = 0;
  double res_ = 0;
  std::vector<float> heights_;
};

#endif  // VEHICLE_SIMULATOR_TERRAIN_MAP_H
