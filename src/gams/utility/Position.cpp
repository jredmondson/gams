/**
 * Copyright(c) 2014 Carnegie Mellon University. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following acknowledgments and disclaimers.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. The names "Carnegie Mellon University," "SEI" and/or "Software
 *    Engineering Institute" shall not be used to endorse or promote products
 *    derived from this software without prior written permission. For written
 *    permission, please contact permission@sei.cmu.edu.
 * 
 * 4. Products derived from this software may not be called "SEI" nor may "SEI"
 *    appear in their names without prior written permission of
 *    permission@sei.cmu.edu.
 * 
 * 5. Redistributions of any form whatsoever must retain the following
 *    acknowledgment:
 * 
 *      This material is based upon work funded and supported by the Department
 *      of Defense under Contract No. FA8721-05-C-0003 with Carnegie Mellon
 *      University for the operation of the Software Engineering Institute, a
 *      federally funded research and development center. Any opinions,
 *      findings and conclusions or recommendations expressed in this material
 *      are those of the author(s) and do not necessarily reflect the views of
 *      the United States Department of Defense.
 * 
 *      NO WARRANTY. THIS CARNEGIE MELLON UNIVERSITY AND SOFTWARE ENGINEERING
 *      INSTITUTE MATERIAL IS FURNISHED ON AN "AS-IS" BASIS. CARNEGIE MELLON
 *      UNIVERSITY MAKES NO WARRANTIES OF ANY KIND, EITHER EXPRESSED OR
 *      IMPLIED, AS TO ANY MATTER INCLUDING, BUT NOT LIMITED TO, WARRANTY OF
 *      FITNESS FOR PURPOSE OR MERCHANTABILITY, EXCLUSIVITY, OR RESULTS
 *      OBTAINED FROM USE OF THE MATERIAL. CARNEGIE MELLON UNIVERSITY DOES
 *      NOT MAKE ANY WARRANTY OF ANY KIND WITH RESPECT TO FREEDOM FROM PATENT,
 *      TRADEMARK, OR COPYRIGHT INFRINGEMENT.
 * 
 *      This material has been approved for public release and unlimited
 *      distribution.
 **/
#include <cmath>
#include <sstream>
#include "gams/utility/Position.h"
#include "gams/utility/GPSPosition.h"
#include "gams/loggers/GlobalLogger.h"

gams::utility::Position::Position(
  double init_x, double init_y, double init_z)
: x(init_x), y(init_y), z(init_z)
{
}

gams::utility::Position::Position(const Position & source)
: x(source.x), y(source.y), z(source.z)
{
}

gams::utility::Position::~Position()
{
}

void
gams::utility::Position::operator=(const Position & rhs)
{
  if (this != &rhs)
  {
    this->x = rhs.x;
    this->y = rhs.y;
    this->z = rhs.z;
  }
}

bool
gams::utility::Position::operator==(const Position & rhs) const
{
  return this->x == rhs.x && this->y == rhs.y && this->z == rhs.z;
}

bool
gams::utility::Position::operator==(
  const madara::knowledge::containers::DoubleArray & rhs) const
{
  return rhs.size() == 3 && 
    this->x == rhs[0] && this->y == rhs[1] && this->z == rhs[2];
}

bool
gams::utility::Position::operator==(
  const madara::knowledge::containers::NativeDoubleArray & rhs) const
{
  return rhs.size() == 3 && 
    this->x == rhs[0] && this->y == rhs[1] && this->z == rhs[2];
}

bool
gams::utility::Position::operator!=(const Position & rhs) const
{
  return !(*this == rhs);
}

bool
gams::utility::Position::operator!=(
  const madara::knowledge::containers::DoubleArray & rhs) const
{
  return !(*this == rhs);
}

bool
gams::utility::Position::operator!=(
  const madara::knowledge::containers::NativeDoubleArray & rhs) const
{
  return !(*this == rhs);
}

bool
gams::utility::Position::operator<(const Position& rhs) const
{
  if (this->x < rhs.x)
    return true;
  else if (this->x == rhs.x)
  {
    if (this->y < rhs.y)
      return true;
    else if (this->y == rhs.y)
      return this->z < rhs.z;
  }
  return false;
}

double
gams::utility::Position::dot(const Position& p_2) const
{
  return this->x * p_2.x + this->y * p_2.y + this->z * p_2.z;
}

bool
gams::utility::Position::approximately_equal(const Position & rhs,
  const double & epsilon) const
{
  return this->distance_to(rhs) <= epsilon;
}

bool
gams::utility::Position::approximately_equal_2d(const Position & rhs,
  const double & epsilon) const
{
  return this->distance_to_2d(rhs) <= epsilon;
}

void
gams::utility::Position::direction_to(const Position& rhs, 
  double& phi, double& theta) const
{
  double x_dist = rhs.x - x;
  double y_dist = rhs.y - y;
  double z_dist = rhs.z - z;
  double dist = pow(pow(x_dist, 2) + pow(y_dist, 2) + pow(z_dist, 2), 0.5);

  theta = acos(z_dist / dist);
  phi = atan2(y_dist, x_dist);
}

double
gams::utility::Position::distance_to(const Position & rhs) const
{
  double sum = pow(this->x - rhs.x, 2.0);
  sum += pow(this->y - rhs.y, 2.0);
  sum += pow(this->z - rhs.z, 2.0);
  return pow(sum, 0.5);
}

double
gams::utility::Position::distance_to_2d(const Position& rhs) const
{
  double sum = pow(this->x - rhs.x, 2.0);
  sum += pow(this->y - rhs.y, 2.0);
  return pow(sum, 0.5);
}

double
gams::utility::Position::distance_to_2d(const Position& end, const Position& check)
  const
{
  // taken from
  // https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
  double l_2 = pow(this->x - end.x, 2.0) + pow(this->y - end.y, 2.0);

  if (l_2 == 0.0) // *this == end
    return distance_to_2d(check);

  const Position p_1 = check - *this;
  const Position p_2 = end - *this;
  double t = p_1.dot(p_2) / l_2;
  if (t < 0.0) // beyond *this end of segment
    return distance_to_2d(check);
  else if (t > 1.0) // beyond end end of segment
    return end.distance_to_2d(check);
  Position projection = *this +(end - *this) * t;
  return check.distance_to_2d(projection);
}

bool
gams::utility::Position::slope_2d(const Position & p, double & slope) const
{
  // check change in x
  double del_x = this->x - p.x;
  if (del_x == 0)
    return false;

  // calculate slope
  slope =(this->y - p.y) / del_x;
  return true;
}

bool
gams::utility::Position::is_between_2d(const Position & end,
  const Position & check) const
{
  // TODO: tune approximate difference parameters
  // check slopes
  double slope_1, slope_2;
  if (this->slope_2d(end, slope_1)) // if not vertical line
  {
    // if vertical line or slopes are different
    if (!this->slope_2d(check, slope_2) ||(std::abs(slope_1 - slope_2) > 0.0001))
      return false;
    if (slope_1 == 0 || slope_2 == 0) // ensure y are the same
    {
      if (std::abs(check.y - end.y) > 0.0001)
        return false;
      return std::abs(check.x - end.x) < 0.0001;
    }
  }
  else // vertical line
  {
    // if not vertical line or x are not the same
    if (this->slope_2d(check, slope_2) ||(std::abs(check.x - this->x) > 0.0001))
      return false;
  }

  // slopes are the same, so just check if x or y is within bounding box
  //    use y
  const bool in_bounding_box =(end.y >= check.y && check.y >= this->y) ||
   (end.y <= check.y && check.y <= this->y);
  return in_bounding_box;
}

std::string
gams::utility::Position::to_string(const std::string & delimiter) const
{
  std::stringstream buffer;
  buffer << x;
  buffer << delimiter;
  buffer << y;
  buffer << delimiter;
  buffer << z;

  return buffer.str();
}

gams::utility::Position
gams::utility::Position::from_string(const std::string & s)
{
  Position temp;
  if (sscanf(s.c_str(), "%lf%*[^0-9]%lf%*[^0-9]%lf", &temp.x, &temp.y, &temp.z) != 3)
    temp.x = temp.y = temp.z = DBL_MAX;
  return temp;
}

void
gams::utility::Position::to_container(
  madara::knowledge::containers::DoubleArray & target) const
{
  target.set(0, x);
  target.set(1, y);
  target.set(2, z);
}

gams::utility::Position
gams::utility::Position::from_record(
  const madara::knowledge::KnowledgeRecord & record)
{
  Position result;

  std::vector <double> values = record.to_doubles();

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "Position::from_record" \
    " reading from record [%s]\n",
    record.to_string().c_str());

  if (values.size() >= 2)
  {
    result.x = values[0];
    result.y = values[1];

    if (values.size() >= 3)
    {
      result.z = values[2];
    }

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MINOR,
      "Position::from_record" \
      " x = %f, y = %f, z = %f\n",
      result.x, result.y, result.z);

  }
  else
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MINOR,
      "Position::from_record" \
      " record was not the appropriate array size(%d instead of >= 2)\n",
     (int)values.size());

  }

  return result;
}

void
gams::utility::Position::from_container(
  madara::knowledge::containers::DoubleArray & source)
{
  if (source.size() >= 3)
  {
    x = source[0];
    y = source[1];
    z = source[2];
  }
}

void
gams::utility::Position::to_container(
  madara::knowledge::containers::NativeDoubleArray & target) const
{
  target.set(0, x);
  target.set(1, y);
  target.set(2, z);
}

void
gams::utility::Position::from_container(
  madara::knowledge::containers::NativeDoubleArray & source)
{
  if (source.size() >= 3)
  {
    x = source[0];
    y = source[1];
    z = source[2];
  }
}

gams::utility::Position
gams::utility::Position::operator-(const Position & rhs) const
{
  return Position(this->x - rhs.x, this->y - rhs.y, this->z - rhs.z);
}

gams::utility::Position
gams::utility::Position::operator+(const Position & rhs) const
{
  return Position(this->x + rhs.x, this->y + rhs.y, this->z + rhs.z);
}

gams::utility::Position
gams::utility::Position::operator*(const double& scale) const
{
  return Position(this->x * scale, this->y * scale, this->z * scale);
}
