
#include "modules/slam_gmapping/slam_gmapping_component.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/common/file.h"
#include "cyber/time/time.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/util.h"
#include "modules/slam_gmapping/common/slam_gmapping_gflags.h"


using apollo::common::ErrorCode;
using apollo::cyber::Time;
using apollo::cyber::class_loader::ClassLoader;

namespace apollo {
namespace slamg_mapping {
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))



SlamGmappingComponent::SlamGmappingComponent()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::SLAM_GMAPPING) {}

SlamGmappingComponent::~SlamGmappingComponent()
{
    if(transform_thread_){
        transform_thread_->join();
    }

    delete gsp_;
    delete gsp_laser_;
    delete gsp_odom_;
}


std::string SlamGmappingComponent::Name() const { return FLAGS_slam_gmapping_module_name; }

bool SlamGmappingComponent::Init() {

    if (!GetProtoConfig(&config_)) {
        AERROR << "Unable to load slam gmapping conf file: " << ConfigFilePath();
        return false;
    }
    tf2_broadcaster_.reset(new apollo::transform::TransformBroadcaster(node_));
    seed_ = static_cast<unsigned long>(cyber::Clock::Now().ToNanosecond());
    gsp_ = new GMapping::GridSlamProcessor();

    gsp_laser_ = nullptr;
    gsp_odom_ = nullptr;
    got_first_scan_ = false;
    got_map_ = false;

    throttle_scans_ = 1;
    base_frame_ = "base_footprint";//base_link
    map_frame_ = "map";
    odom_frame_ = "odom_combined";
    transform_publish_period_ = 0.05;

    map_update_interval_ = 0.5;
    maxUrange_ = 80.0;  maxRange_ = 0.0;
    minimum_score_ = 0;
    sigma_ = 0.05;
    kernelSize_ = 1;
    lstep_ = 0.05;
    astep_ = 0.05;
    iterations_ = 5;
    lsigma_ = 0.075;
    ogain_ = 3.0;
    lskip_ = 0;
    srr_ = 0.1;
    srt_ = 0.2;
    str_ = 0.1;
    stt_ = 0.2;
    linearUpdate_ = 1.0;
    angularUpdate_ = 0.5;
    temporalUpdate_ = 1.0;
    resampleThreshold_ = 0.5;
    particles_ = 30;
    xmin_ = -10.0;
    ymin_ = -10.0;
    xmax_ = 10.0;
    ymax_ = 10.0;
    delta_ = 0.05;
    occ_thresh_ = 0.25;
    llsamplerange_ = 0.01;
    llsamplestep_ = 0.01;
    lasamplerange_ = 0.005;
    lasamplestep_ = 0.005;
    tf_delay_ = transform_publish_period_;

    startLiveSlam();
    return true;
}

void SlamGmappingComponent::startLiveSlam() {
    entropy_writer_ = node_->CreateWriter<apollo::akman::Etropy>("apollo/slam_gmapping/entropy");
    sst_writer_     = node_->CreateWriter<apollo::akman::OccupancyGrid>("apollo/slam_gmapping/map");
    sstm_writer_    = node_->CreateWriter<apollo::akman::MapMetaData>("apollo/slam_gmapping/map_metadata");
    laser_reader_   = node_->CreateReader<apollo::akman::LaserScan>(
            config_.laser_scan_channel_name(), 
            [this](const std::shared_ptr<apollo::akman::LaserScan>& scan) {
                laserCallback(scan);
            });
    transform_thread_ = std::make_shared<std::thread>
            (std::bind(&SlamGmappingComponent::publishLoop, this, transform_publish_period_));

}

void SlamGmappingComponent::publishTransform() {
    map_to_odom_mutex_.lock();
    double tf_expiration = cyber::Clock::Now().ToSecond() + tf_delay_;
    apollo::transform::TransformStamped transform;
    transform.mutable_header()->set_frame_id(map_frame_);
    transform.mutable_header()->set_timestamp_sec(tf_expiration);

    
    transform.mutable_transform()->mutable_translation()->set_x(map_to_odom_.getOrigin().getX());
    transform.mutable_transform()->mutable_translation()->set_y(map_to_odom_.getOrigin().getY());
    transform.mutable_transform()->mutable_translation()->set_z(map_to_odom_.getOrigin().getZ());

    transform.mutable_transform()->mutable_rotation()->set_qx(map_to_odom_.getRotation().getX());
    transform.mutable_transform()->mutable_rotation()->set_qy(map_to_odom_.getRotation().getY());
    transform.mutable_transform()->mutable_rotation()->set_qz(map_to_odom_.getRotation().getZ());
    transform.mutable_transform()->mutable_rotation()->set_qw(map_to_odom_.getRotation().getW());

    tf2_broadcaster_->SendTransform(transform);

    map_to_odom_mutex_.unlock();


}

bool SlamGmappingComponent::initMapper(const apollo::akman::LaserScan& scan)
{
    laser_frame_ = scan.header().frame_id();
    // Get the laser's pose, relative to base.
    apollo::akman::PoseStamped ident; //雷达初始位姿 
    apollo::akman::PoseStamped laser_pose; //雷达相对于基础坐标系的位姿

    apollo::transform::TransformStamped stamped_transform;
    StampedTransform trans;
    auto query_time = apollo::cyber::Time(0);
    try {
    stamped_transform =
        tf2_buffer_->lookupTransform(base_frame_, scan.header().frame_id(), query_time);

    // trans.translation =
    //     Eigen::Translation3d(stamped_transform.transform().translation().x(),
    //                             stamped_transform.transform().translation().y(),
    //                             stamped_transform.transform().translation().z());
    // trans.rotation =
    //     Eigen::Quaterniond(stamped_transform.transform().rotation().qw(),
    //                         stamped_transform.transform().rotation().qx(),
    //                         stamped_transform.transform().rotation().qy(),
    //                         stamped_transform.transform().rotation().qz());

    //to do transform   ident  坐标系转换; laser_frame_ --> base_frame_
    TransformPoseStamped(ident, laser_pose, stamped_transform);

    } catch (tf2::TransformException& ex) {
    AERROR << ex.what();
    return false;
    }

    // try{
    //     ident.mutable_header()->set_frame_id(laser_frame_);
    //     ident.mutable_header()->set_timestamp_sec(scan.header().timestamp_sec());

    //     buffer_->transform(ident, laser_pose, base_frame_); //坐标系转换; laser_frame_ --> base_frame_
    // }
    // catch (tf2::TransformException& e){
    //     RCLCPP_WARN(this->get_logger(), "Failed to compute laser pose, aborting initialization (%s)", e.what());
    //     return false;
    // }

    // create a point 1m above the laser position and transform it into the laser-frame
    apollo::akman::PointStamped up;
    up.mutable_header()->set_timestamp_sec(scan.header().timestamp_sec());
    up.mutable_header()->set_frame_id(laser_frame_);
    up.mutable_point()->set_x(0);
    up.mutable_point()->set_y(0);
    up.mutable_point()->set_z(1 + laser_pose.pose().position().z());

    // geometry_msgs::msg::PointStamped up;
    // up.header.stamp = scan->header.stamp;
    // up.header.frame_id = base_frame_;
    // up.point.x = up.point.y = 0;
    // up.point.z = 1 + laser_pose.pose.position.z;
    try
    {
        stamped_transform =
            tf2_buffer_->lookupTransform(scan.header().frame_id(),base_frame_, query_time);

        //todo   坐标系转换; base_frame_ --> laser_frame_
        TransformPointStamped(up, up, stamped_transform);

        //buffer_->transform(up, up, laser_frame_); //坐标系转换; base_frame_ --> laser_frame_
    }
    catch(tf2::TransformException& e)
    {
        AERROR << "Unable to determine orientation of laser: %s"<<e.what();
        return false;
    }

    // gmapping doesnt take roll or pitch into account. So check for correct sensor alignment.
    if (fabs(fabs(up.point().z()) - 1) > 0.001)
    {
        AERROR<<"Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: "<<up.point().z();
        return false;
    }

    gsp_laser_beam_count_ = static_cast<unsigned int>(scan.ranges().size());

    double angle_center = (scan.angle_min() + scan.angle_max())/2; //激光扫描中心角度

    centered_laser_pose_.mutable_header()->set_frame_id(laser_frame_);
    centered_laser_pose_.mutable_header()->set_timestamp_sec(apollo::cyber::Clock::Now().ToSecond());
    tf2::Quaternion q;  //(x, y, z, w)

    //判断激光雷达的安装朝向
    if (up.point().z() > 0)
    {
        do_reverse_range_ = scan.angle_min() > scan.angle_max();
        q.setEuler(angle_center, 0, 0);  //设置绕着y轴的旋转角度为angle_center？
        AINFO << "Laser is mounted upwards.";
    }
    else
    {
        do_reverse_range_ = scan.angle_min()  < scan.angle_max();
        q.setEuler(-angle_center, 0, M_PI);
        AINFO << "Laser is mounted upside down.";
    }

    //激光传感器的位置坐标
    centered_laser_pose_.mutable_pose()->mutable_position()->set_x(0);
    centered_laser_pose_.mutable_pose()->mutable_position()->set_y(0);
    centered_laser_pose_.mutable_pose()->mutable_position()->set_z(0);
    
    //激光传感器方向信息：q = w + xi + yj + zk   w:旋转角度； x、y、z:旋转轴的方向矢量
    centered_laser_pose_.mutable_pose()->mutable_orientation()->set_qw(q.getW());
    centered_laser_pose_.mutable_pose()->mutable_orientation()->set_qx(q.getX());
    centered_laser_pose_.mutable_pose()->mutable_orientation()->set_qy(q.getY());
    centered_laser_pose_.mutable_pose()->mutable_orientation()->set_qz(q.getZ());

    // Compute the angles of the laser from -x to x, basically symmetric and in increasing order
    laser_angles_.resize(scan.ranges().size());
    // Make sure angles are started so that they are centered
    double theta = - std::fabs(scan.angle_min() - scan.angle_max())/2;
    for(unsigned int i=0; i<scan.ranges().size(); ++i)
    {
        laser_angles_[i]=theta;
        theta += std::fabs(scan.angle_increment());
    }

    AINFO << "Laser angles in laser-frame: min: "<<scan.angle_min() 
          << "max: "<< scan.angle_max()
          << "inc: "<< scan.angle_increment();
    AINFO << "Laser angles in top-down centered laser-frame: min: " << laser_angles_.front()
          << "max: "<< laser_angles_.back()
          <<"inc: "<< std::fabs(scan.angle_increment());

    GMapping::OrientedPoint gmap_pose(0, 0, 0);  //(x, y, theta)

    // setting maxRange and maxUrange here so we can set a reasonable default
    maxRange_ = scan.range_max() - 0.01;
    maxUrange_ = maxRange_;

    // The laser must be called "FLASER".
    // We pass in the absolute value of the computed angle increment, on the
    // assumption that GMapping requires a positive angle increment.  If the
    // actual increment is negative, we'll swap the order of ranges before
    // feeding each scan to GMapping.
    gsp_laser_ = new GMapping::RangeSensor("FLASER", gsp_laser_beam_count_, fabs(scan.angle_increment()), gmap_pose,
                                           0.0, maxRange_);

    GMapping::SensorMap smap;
    smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
    gsp_->setSensorMap(smap);

    gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);

    /// @todo Expose setting an initial pose
    GMapping::OrientedPoint initialPose;
    if(!getOdomPose(initialPose, scan.header().timestamp_sec()))
    {
        AWARN << "Unable to determine inital pose of laser! Starting point will be set to zero.";
        initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
    }

    gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,
                                kernelSize_, lstep_, astep_, iterations_,
                                lsigma_, ogain_, static_cast<unsigned int>(lskip_));

    gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
    gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
    gsp_->setUpdatePeriod(temporalUpdate_);
    gsp_->setgenerateMap(false);
    gsp_->GridSlamProcessor::init(static_cast<unsigned int>(particles_), xmin_, ymin_, xmax_, ymax_,
                                  delta_, initialPose);
    gsp_->setllsamplerange(llsamplerange_);
    gsp_->setllsamplestep(llsamplestep_);
    /// @todo Check these calls; in the gmapping gui, they use
    /// llsamplestep and llsamplerange intead of lasamplestep and
    /// lasamplerange.  It was probably a typo, but who knows.
    gsp_->setlasamplerange(lasamplerange_);
    gsp_->setlasamplestep(lasamplestep_);
    gsp_->setminimumScore(minimum_score_);

    // Call the sampling function once to set the seed.
    GMapping::sampleGaussian(1, static_cast<unsigned int>(seed_));

    AINFO << "Initialization complete";

    return true;
}




double SlamGmappingComponent::computePoseEntropy()
{
    double weight_total=0.0;
    for (const auto &it : gsp_->getParticles()) {
        weight_total += it.weight;
    }
    double entropy = 0.0;
    for (const auto &it : gsp_->getParticles()) {
        if(it.weight/weight_total > 0.0)
            entropy += it.weight/weight_total * std::log(it.weight/weight_total);
    }
    return -entropy;
}

bool SlamGmappingComponent::getOdomPose(GMapping::OrientedPoint& gmap_pose, const double& t)
{
    // Get the pose of the centered laser at the right time
    centered_laser_pose_.mutable_header()->set_timestamp_sec(t);
    // Get the laser's pose that is centered
    apollo::akman::PoseStamped odom_pose;

    apollo::transform::TransformStamped stamped_transform;
    StampedTransform trans;
    auto query_time = apollo::cyber::Time(1.0);
    try {
    stamped_transform =
        tf2_buffer_->lookupTransform(odom_frame_, centered_laser_pose_.header().frame_id(), query_time);
    TransformPoseStamped(centered_laser_pose_, odom_pose, stamped_transform);

    // trans.translation =
    //     Eigen::Translation3d(stamped_transform.transform().translation().x(),
    //                             stamped_transform.transform().translation().y(),
    //                             stamped_transform.transform().translation().z());
    // trans.rotation =
    //     Eigen::Quaterniond(stamped_transform.transform().rotation().qw(),
    //                         stamped_transform.transform().rotation().qx(),
    //                         stamped_transform.transform().rotation().qy(),
    //                         stamped_transform.transform().rotation().qz());
    }
   // geometry_msgs::msg::PoseStamped odom_pose;
    // try
    // {
    //     buffer_->transform(centered_laser_pose_, odom_pose, odom_frame_, tf2::durationFromSec(1.0));
    // }
    catch(tf2::TransformException& e)
    {
        AWARN << "Failed to compute odom pose, skipping scan (%s)", e.what();
        return false;
    }
    

    double yaw = getYaw(odom_pose.pose().orientation());

    gmap_pose = GMapping::OrientedPoint(odom_pose.pose().position().x(),
                                        odom_pose.pose().position().y(),
                                        yaw);
    return true;
}



bool SlamGmappingComponent::addScan(
    const std::shared_ptr<apollo::akman::LaserScan>& scan, 
    GMapping::OrientedPoint& gmap_pose) {
    if (!getOdomPose(gmap_pose, scan->header().timestamp_sec()))
        return false;

    if (scan->ranges().size() != gsp_laser_beam_count_)
        return false;

    // GMapping wants an array of doubles...
    auto *ranges_double = new double[scan->ranges().size()];
    // If the angle increment is negative, we have to invert the order of the readings.
    if (do_reverse_range_) {
        ADEBUG << "Inverting scan";
        int num_ranges = static_cast<int>(scan->ranges().size());
        for (int i = 0; i < num_ranges; i++) {
            // Must filter out short readings, because the mapper won't
            if (scan->ranges(num_ranges - i - 1) < scan->range_min())
                ranges_double[i] = (double) scan->range_max();
            else
                ranges_double[i] = (double) scan->ranges(num_ranges - i - 1);
        }
    } else {
        for (unsigned int i = 0; i < scan->ranges().size(); i++) {
            // Must filter out short readings, because the mapper won't
            if (scan->ranges(i) < scan->range_min())
                ranges_double[i] = (double) scan->range_max();
            else
                ranges_double[i] = (double) scan->ranges(i);
        }
    }

    GMapping::RangeReading reading(static_cast<unsigned int>(scan->ranges().size()),
                                   ranges_double,
                                   gsp_laser_,
                                   scan->header().timestamp_sec());

    // ...but it deep copies them in RangeReading constructor, so we don't
    // need to keep our array around.
    delete[] ranges_double;

    reading.setPose(gmap_pose);

    ADEBUG << "processing scan";

    return gsp_->processScan(reading);
}

void SlamGmappingComponent::updateMap(const std::shared_ptr<apollo::akman::LaserScan>& scan)
{
    AINFO << "Update Map Start";
    map_mutex_.lock();
    GMapping::ScanMatcher matcher;

    matcher.setLaserParameters(static_cast<unsigned int>(scan->ranges().size()), &(laser_angles_[0]),
                               gsp_laser_->getPose());

    matcher.setlaserMaxRange(maxRange_);
    matcher.setusableRange(maxUrange_);
    matcher.setgenerateMap(true);

    GMapping::GridSlamProcessor::Particle best =
            gsp_->getParticles()[gsp_->getBestParticleIndex()];
    apollo::akman::Etropy entropy;
    entropy.set_entropy(computePoseEntropy());
    if(entropy.entropy() > 0.0)
        entropy_writer_->Write(entropy);

    if(!got_map_) {
        map_.mutable_meta_data()->set_resolution(delta_);
        map_.mutable_meta_data()->mutable_origin()->mutable_position()->set_x(0.0);
        map_.mutable_meta_data()->mutable_origin()->mutable_position()->set_y(0.0);
        map_.mutable_meta_data()->mutable_origin()->mutable_position()->set_z(0.0);
        map_.mutable_meta_data()->mutable_origin()->mutable_orientation()->set_qx(0.0);
        map_.mutable_meta_data()->mutable_origin()->mutable_orientation()->set_qy(0.0);
        map_.mutable_meta_data()->mutable_origin()->mutable_orientation()->set_qz(0.0);
        map_.mutable_meta_data()->mutable_origin()->mutable_orientation()->set_qw(1.0);
    }

    GMapping::Point center;
    center.x=(xmin_ + xmax_) / 2.0;
    center.y=(ymin_ + ymax_) / 2.0;

    GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_,
                                  delta_);

    AINFO << "Trajectory tree:";
    for(GMapping::GridSlamProcessor::TNode* n = best.node;
        n;
        n = n->parent)
    {
        AINFO << "x: " << n->pose.x << " y: " << n->pose.y << " theta: " << n->pose.theta;
        if(!n->reading)
        {
            AINFO << "Reading is NULL";
            continue;
        }
        matcher.invalidateActiveArea();
        matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
        matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
    }

    // the map may have expanded, so resize ros message as well
    if(map_.meta_data().width() != (unsigned int) smap.getMapSizeX() || 
        map_.meta_data().height() != (unsigned int) smap.getMapSizeY()) {

        // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
        //       so we must obtain the bounding box in a different way
        GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
        GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
        xmin_ = wmin.x; ymin_ = wmin.y;
        xmax_ = wmax.x; ymax_ = wmax.y;

        AINFO << "map size is now, x:"<<smap.getMapSizeX()<< " y: " <<smap.getMapSizeY() 
        << " pixels: (" <<xmin_<<", "<<ymin_<< "-" << "( "<< xmax_<< ", " <<  ymax_ <<")";

        map_.mutable_meta_data()->set_width(smap.getMapSizeX());
        map_.mutable_meta_data()->set_height(smap.getMapSizeY());
        map_.mutable_meta_data()->mutable_origin()->mutable_position()->set_x(xmin_);
        map_.mutable_meta_data()->mutable_origin()->mutable_position()->set_x(ymin_);
        map_.mutable_data()->Reserve(map_.meta_data().width() * map_.meta_data().height()); 

        AINFO << "map origin: (%f, " << map_.meta_data().origin().position().x() 
              << " "<<map_.meta_data().origin().position().y()<<")";
    }

    for(int x=0; x < smap.getMapSizeX(); x++)
    {
        for(int y=0; y < smap.getMapSizeY(); y++)
        {
            /// @todo Sort out the unknown vs. free vs. obstacle thresholding
            GMapping::IntPoint p(x, y);
            double occ=smap.cell(p);
            assert(occ <= 1.0);
            if(occ < 0)
                map_.set_data(static_cast<int>(MAP_IDX(map_.meta_data().width(), x, y)), -1);
            else if(occ > occ_thresh_)
            {
                //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
                map_.set_data(static_cast<int>(MAP_IDX(map_.meta_data().width(), x, y)), 100);
            }
            else
                map_.set_data(static_cast<int>(MAP_IDX(map_.meta_data().width(), x, y)), 0);
        }
    }
    got_map_ = true;

    //make sure to set the header information on the map
    map_.mutable_header()->set_timestamp_sec(cyber::Clock::Now().ToSecond());
    map_.mutable_header()->set_frame_id(map_frame_);

    sst_writer_->Write(map_);
    sstm_writer_->Write(map_.meta_data());
    map_mutex_.unlock();
}

void SlamGmappingComponent::laserCallback(const std::shared_ptr<apollo::akman::LaserScan>& scan) {
    laser_count_++;
    if ((laser_count_ % throttle_scans_) != 0)
        return;
    
    last_map_update_ = cyber::Clock::Now().ToSecond();

    // We can't initialize the mapper until we've got the first scan
    if(!got_first_scan_)
    {
        if(!initMapper(*scan))
            return;
        got_first_scan_ = true;
    }

    GMapping::OrientedPoint odom_pose;

    if(addScan(scan, odom_pose))
    {
        GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;

        tf2::Quaternion q;
        q.setRPY(0, 0, mpose.theta);
        tf2::Transform laser_to_map = tf2::Transform(q, tf2::Vector3(mpose.x, mpose.y, 0.0)).inverse();
        q.setRPY(0, 0, odom_pose.theta);
        tf2::Transform odom_to_laser = tf2::Transform(q, tf2::Vector3(odom_pose.x, odom_pose.y, 0.0));

        map_to_odom_mutex_.lock();
        map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
        map_to_odom_mutex_.unlock();

        auto timestamp = scan->header().timestamp_sec();
        if(!got_map_ || (timestamp - last_map_update_) > map_update_interval_)
        {
            updateMap(scan);
            last_map_update_ = scan->header().timestamp_sec();
        }
    }


}



void SlamGmappingComponent::publishLoop(double transform_publish_period) {
    if (transform_publish_period == 0)
        return;
    cyber::Rate r(1.0 / transform_publish_period);
    while (cyber::OK()) {
        publishTransform();
        r.Sleep();
    }

}

}
};