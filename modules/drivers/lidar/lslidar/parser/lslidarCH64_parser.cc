/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/drivers/lidar/lslidar/parser/lslidar_parser.h"

namespace apollo {
namespace drivers {
namespace lslidar {

LslidarCH64Parser::LslidarCH64Parser(const Config& config)
    : LslidarParser(config), previous_packet_stamp_(0), gps_base_usec_(0) {
      scan_points_.resize(50);
}

//产生点云数据
void LslidarCH64Parser::GeneratePointcloud(
    const std::shared_ptr<LslidarScan>& scan_msg,
    const std::shared_ptr<PointCloud>& out_msg, 
    const std::shared_ptr<cyber::Writer<apollo::akman::LaserScan>>& laser_scan_writer) {
  // allocate a point cloud with same time and frame ID as raw data
  out_msg->mutable_header()->set_timestamp_sec(scan_msg->basetime() /
                                               1000000000.0);
  out_msg->mutable_header()->set_module_name(scan_msg->header().module_name());
  out_msg->mutable_header()->set_frame_id(scan_msg->header().frame_id());
  out_msg->set_height(1);
  out_msg->set_measurement_time(scan_msg->basetime() / 1000000000.0);
  out_msg->mutable_header()->set_sequence_num(
      scan_msg->header().sequence_num());
  gps_base_usec_ = scan_msg->basetime();

  packets_size = scan_msg->firing_pkts_size();

  for (size_t i = 0; i < packets_size; ++i) {
    Unpack(static_cast<int>(i), scan_msg->firing_pkts(static_cast<int>(i)),
           out_msg, laser_scan_writer);
    last_time_stamp_ = out_msg->measurement_time();
    ADEBUG << "stamp: " << std::fixed << last_time_stamp_;
  }

  if (out_msg->point().empty()) {
    // we discard this pointcloud if empty
    AERROR << "All points is NAN!Please check lslidar:" << config_.model();
  }

  // set default width
  out_msg->set_width(out_msg->point_size());
}

/** @brief convert raw packet to point cloud
 *  @param pkt raw packet to Unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void LslidarCH64Parser::Unpack(int num, const LslidarPacket& pkt,
                               std::shared_ptr<PointCloud> pc,
                               const std::shared_ptr<cyber::Writer<apollo::akman::LaserScan>>& laser_scan_writer) {
  float x, y, z;
  uint64_t point_time;
  //uint64_t packet_end_time;
  double z_sin_altitude = 0.0;
  double z_cos_altitude = 0.0;
  time_last = 0;
  const RawPacket* raw = (const RawPacket*)pkt.data().c_str();

  //packet_end_time = pkt.stamp();
  uint8_t *data = reinterpret_cast<uint8_t *>(
                  const_cast<char *>(pkt.data().c_str()));
  // for (int i = 0; i < 108; i++) {
  //         std::cout << std::hex<<std::setw(2)<<std::setfill('0')<<(int)data[i]<<" ";
  // }
  // std::cout<<std::endl;

  data_processing(data, 6);
  if (count_num != 0) {
    PubLaserScan(laser_scan_writer);
  }
  
  pc->mutable_header()->set_timestamp_sec(apollo::cyber::Time().Now().ToSecond());
  pc->set_frame_id(config_.frame_id());
  pc->set_height(1);
  double timestamp = pre_time_.ToSecond();
  double scan_time = time_.ToSecond() - pre_time_.ToSecond();
  int width = 0;
  for (uint16_t i = 0; i < count_num; i++) {
    double degree = 360.0 - scan_points_[i].degree;
    bool pass_point = false;
    if (config_.angle_able_max() < 360) {
      if (degree < config_.angle_able_min() || degree > config_.angle_able_max()) {
				pass_point = true;
      }
    } else {
      if (degree < config_.angle_able_min() && degree > (config_.angle_able_max() - 360)) {
				pass_point = true;
      }
    }

    if (scan_points_[i].range < 0.001) {
        pass_point = true;
    }
    if (!pass_point)
    {
      // printf("degree = %f\n",degree);
      // printf("angle_able_min = %f\nangle_able_max=%f\n",angle_able_min,angle_able_max);
      PointXYZIT* point = pc->add_point();
      int point_idx = round(degree * count_num / 360);
      point->set_timestamp(timestamp - point_idx * (scan_time / count_num));
      // printf("timestamp = %f\n",point.timestamp);
      point->set_x(scan_points_[i].range * cos(M_PI / 180 * scan_points_[i].degree));
      point->set_y(-scan_points_[i].range * sin(M_PI / 180 * scan_points_[i].degree));
      point->set_z(0);
      point->set_intensity(scan_points_[i].intensity);
      ++width;
    }
    if (scan_points_[i + 25].range < 0.001)
      pass_point = true;
    if (!pass_point)
    {
      // printf("degree = %f\n",degree);
      // printf("angle_able_min = %f\nangle_able_max=%f\n",angle_able_min,angle_able_max);
      PointXYZIT* point = pc->add_point();
      int point_idx = round(degree * count_num / 360);
      point->set_timestamp(timestamp - point_idx * (scan_time / count_num));
      // printf("timestamp = %f\n",point.timestamp);
      point->set_x(scan_points_[i + 25].range * cos(M_PI / 180 * scan_points_[i].degree));
      point->set_y(-scan_points_[i + 25].range * sin(M_PI / 180 * scan_points_[i].degree));
      point->set_z(0);
      point->set_intensity(scan_points_[i + 25].intensity);
      ++width;
    }
  }
  pc->set_width(width);
  count_num = 0;
  for (long unsigned int k = 0; k < scan_points_.size(); k++)
  {
    scan_points_[k].range = 0;
    scan_points_[k].degree = 0;
    scan_points_[k].intensity = 0;
  }

  
  // for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET; point_idx++) {
  //   firings[point_idx].vertical_line = raw->points[point_idx].vertical_line;
  //   two_bytes point_amuzith;
  //   point_amuzith.bytes[0] = raw->points[point_idx].azimuth_2;
  //   point_amuzith.bytes[1] = raw->points[point_idx].azimuth_1;
  //   firings[point_idx].azimuth =
  //       static_cast<double>(point_amuzith.uint) * 0.01 * DEG_TO_RAD;
  //   four_bytes point_distance;
  //   point_distance.bytes[0] = raw->points[point_idx].distance_3;
  //   point_distance.bytes[1] = raw->points[point_idx].distance_2;
  //   point_distance.bytes[2] = raw->points[point_idx].distance_1;
  //   point_distance.bytes[3] = 0;
  //   firings[point_idx].distance =
  //       static_cast<double>(point_distance.uint) * DISTANCE_RESOLUTION2;
  //   firings[point_idx].intensity = raw->points[point_idx].intensity;
  // }

  // for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET; point_idx++) {
  //   LaserCorrection& corrections =
  //       calibration_.laser_corrections_[firings[point_idx].vertical_line];

  //   if (config_.calibration())
  //     firings[point_idx].distance =
  //         firings[point_idx].distance + corrections.dist_correction;

  //   if (firings[point_idx].distance > config_.max_range() ||
  //       firings[point_idx].distance < config_.min_range())
  //     continue;

  //   int line_num = firings[point_idx].vertical_line;

  //   // Convert the point to xyz coordinate
  //   if (line_num % 8 == 0 || line_num % 8 == 1 || line_num % 8 == 2 ||
  //       line_num % 8 == 3) {
  //     z_sin_altitude =
  //         sin(-13.33 * DEG_TO_RAD + floor(line_num / 4) * 1.33 * DEG_TO_RAD) +
  //         2 * cos(firings[point_idx].azimuth / 2 + 1.05 * DEG_TO_RAD) *
  //             sin((line_num % 4) * 0.33 * DEG_TO_RAD);

  //   } else if (line_num % 8 == 4 || line_num % 8 == 5 || line_num % 8 == 6 ||
  //              line_num % 8 == 7) {
  //     z_sin_altitude =
  //         sin(-13.33 * DEG_TO_RAD + floor(line_num / 4) * 1.33 * DEG_TO_RAD) +
  //         2 * cos(firings[point_idx].azimuth / 2 - 1.05 * DEG_TO_RAD) *
  //             sin((line_num % 4) * 0.33 * DEG_TO_RAD);
  //   }

  //   z_cos_altitude = sqrt(1 - z_sin_altitude * z_sin_altitude);
  //   x = firings[point_idx].distance * z_cos_altitude *
  //       cos(firings[point_idx].azimuth);
  //   y = firings[point_idx].distance * z_cos_altitude *
  //       sin(firings[point_idx].azimuth);
  //   z = firings[point_idx].distance * z_sin_altitude;

  //   // Compute the time of the point
    // point_time = packet_end_time - 1726 * (POINTS_PER_PACKET - point_idx - 1);
  //   if (time_last < point_time && time_last > 0) {
  //     point_time = time_last + 1726;
  //   }
  //   time_last = point_time;

  //   PointXYZIT* point = pc->add_point();
  //   point->set_timestamp(point_time);
  //   point->set_intensity(firings[point_idx].intensity);

  //   if (config_.calibration()) {
  //     ComputeCoords2(firings[point_idx].vertical_line, CH64,
  //                    firings[point_idx].distance, &corrections,
  //                    firings[point_idx].azimuth, point);

  //   } else {
  //     if ((y >= config_.bottom_left_x() && y <= config_.top_right_x()) &&
  //         (-x >= config_.bottom_left_y() && -x <= config_.top_right_y())) {
  //       point->set_x(nan);
  //       point->set_y(nan);
  //       point->set_z(nan);
  //       point->set_timestamp(point_time);
  //       point->set_intensity(0);
  //     } else {
  //       point->set_x(y);
  //       point->set_y(-x);
  //       point->set_z(z);
  //     }
  //   }
  // }
}

void LslidarCH64Parser::PubLaserScan(
  const std::shared_ptr<cyber::Writer<apollo::akman::LaserScan>>& laser_scan_writer) {

  auto scan = apollo::akman::LaserScan();
  ////int scan_num = count_num * 2;
  int scan_num = count_num ;

  std::vector<ScanPointN10P> points;
  scan.mutable_header()->set_frame_id(config_.frame_id());

  scan.mutable_header()->set_timestamp_sec(apollo::cyber::Time::Now().ToSecond()); // timestamp will obtained from sweep data stamp
  

  scan.set_angle_min(0);
  scan.set_angle_max(2 * M_PI);
  scan.set_angle_increment(2 * M_PI / (double)(count_num));
  scan.set_range_min(config_.min_range());
  scan.set_range_max(config_.max_range());
  scan.mutable_ranges()->Reserve(scan_num);

  scan.mutable_intensities()->Reserve(scan_num);

  float temp_ranges[count_num];
  float temp_intensities[count_num];

  for (int k = 0; k < scan_num; k++)
  {
    scan.add_ranges(std::numeric_limits<float>::infinity());
    // temp_ranges[k] = std::numeric_limits<float>::infinity();
    // temp_intensities[k] = 0.0;
    scan.add_ranges(0);
  }



  for (int i = 0; i < count_num; i++)
  {
    int point_idx = round((360 - scan_points_[i].degree) * count_num / 360);
    AERROR << "point_idx: "<<point_idx << "count_num: "<<count_num;
    if (scan_points_[i].range == 0.0)
    {
      temp_ranges[point_idx] = std::numeric_limits<float>::infinity();
      temp_intensities[point_idx] = 0.0;
      // scan.mutable_ranges()->Set(point_idx, std::numeric_limits<float>::infinity());
      // scan.mutable_intensities()->Set(point_idx, 0.0);
      // scan.set_ranges(point_idx, std::numeric_limits<float>::infinity());
      // scan.set_intensities(point_idx, 0.0)

    }
    else
    {
      double dist = scan_points_[i].range;

      temp_ranges[point_idx] = (float)dist;
      temp_intensities[point_idx] = scan_points_[i].intensity;
      // scan.mutable_ranges()->Set(point_idx, (float)dist);
      // scan.mutable_intensities()->Set(point_idx, scan_points_[i].intensity);

    }
  }

  for (int i = 0; i < count_num; i++) {
    scan.add_ranges(temp_ranges[i]);
    scan.add_intensities(temp_intensities[i]);
  }
  laser_scan_writer->Write(scan);
}

void LslidarCH64Parser::data_processing(
  unsigned char *packet_bytes, 
  int len) {
		double degree;
		double end_degree;
		double degree_interval = 15.0;
		// boost::posix_time::ptime t1, t2;
		// t1 = boost::posix_time::microsec_clock::universal_time();

		int s = packet_bytes[config_.degree_bits_start()];
		int z = packet_bytes[config_.degree_bits_start() + 1];

		degree = (s * 256 + z) / 100.f + degree_compensation;
		degree = (degree < 0) ? degree + 360 : degree;
		degree = (degree > 360) ? degree - 360 : degree;

    int s_e = packet_bytes[config_.end_degree_bits_start()];
    int z_e = packet_bytes[config_.end_degree_bits_start() + 1];

    end_degree = (s_e * 256 + z_e) / 100.f;
    end_degree = (end_degree > 360) ? end_degree - 360 : end_degree;

    if (degree > end_degree)
      degree_interval = end_degree + 360 - degree;
    else
      degree_interval = end_degree - degree;

		int invalidValue = 0;
		int point_len = 6;

		for (int num = 0; num < point_len * config_.package_points(); num += point_len)
		{
			int s = packet_bytes[num + config_.data_bits_start()];
			int z = packet_bytes[num + config_.data_bits_start() + 1];
			if ((s * 256 + z) == 0xFFFF)
				invalidValue++;
		}

		invalidValue = config_.package_points() - invalidValue;
		
		invalidValue--;
		if (invalidValue <= 1)
		{
			delete packet_bytes;
			return;
		}

		for (int num = 0; num < config_.package_points(); num++)
		{
			int s = packet_bytes[num * point_len + config_.data_bits_start()];
			int z = packet_bytes[num * point_len + config_.data_bits_start() + 1];
			int y = packet_bytes[num * point_len + config_.data_bits_start()  + 2];;
	
			if ((s * 256 + z) != 0xFFFF)
			{
				scan_points_[idx].range = double(s * 256 + (z)) / 1000.f;
				
				scan_points_[idx].intensity = int(y);

				s = packet_bytes[num * point_len + config_.data_bits_start() + point_len / 2];
				z = packet_bytes[num * point_len + config_.data_bits_start() + point_len / 2 + 1];
				
				y = packet_bytes[num * point_len + config_.data_bits_start() + point_len / 2 + 2];

				scan_points_[idx + 25].range = double(s * 256 + (z)) / 1000.f;
				scan_points_[idx + 25].intensity = int(y);
        AWARN<<"------- "<<idx+25<<"   range: "<<double(s * 256 + (z)) / 1000.f<<" intensity:"<<int(y);
				//计算每个点的角度
				if ((degree + (degree_interval / invalidValue * num)) > 360)
					scan_points_[idx].degree = degree + (degree_interval / invalidValue * num) - 360;
				else
					scan_points_[idx].degree = degree + (degree_interval / invalidValue * num);
			} else {
				continue;
      }
      AWARN << "scan_points_["<<idx<<"] "<<scan_points_[idx].degree<< "last_degree"<<last_degree;
      AWARN << "idx "<<idx<< "points_size "<<config_.points_size();
			if (((scan_points_[idx].degree < last_degree && scan_points_[idx].degree < 5 && last_degree > 355) || idx >= config_.points_size()) && idx > 10)
			{
        
				last_degree = scan_points_[idx].degree;
				count_num = idx;
				idx = 0;
				for (int k = 0; k < count_num; k++)
				{
					if (config_.angle_able_max() > 360)
					{
						if ((360 - scan_points_[k].degree) > (config_.angle_able_max() - 360) && (360 - scan_points_[k].degree) < config_.angle_able_min())
						{
							scan_points_[k].range = 0;
							scan_points_[k + 25].range = 0;
						}
					}
					else
					{
						if ((360 - scan_points_[k].degree) > config_.angle_able_max() || (360 - scan_points_[k].degree) < config_.angle_able_min())
						{
							scan_points_[k].range = 0;
							scan_points_[k + 25].range = 0;
						}
					}
					if (scan_points_[k].range < config_.min_range() || scan_points_[k].range > config_.max_range())
						scan_points_[k].range = 0;
					if (scan_points_[k + 25].range < config_.min_range() || scan_points_[k + 25].range > config_.max_range())
						scan_points_[k + 25].range = 0;
				}
				
				pre_time_ = time_;
				time_ = apollo::cyber::Time().Now();
			} else {
				last_degree = scan_points_[idx].degree;
				idx++;
			}
		}
		packet_bytes = {0x00};
		if (packet_bytes)
		{
			packet_bytes = NULL;
			delete packet_bytes;
		}
	}

void LslidarCH64Parser::Order(std::shared_ptr<PointCloud> cloud) {}

}  // namespace lslidar
}  // namespace drivers
}  // namespace apollo
