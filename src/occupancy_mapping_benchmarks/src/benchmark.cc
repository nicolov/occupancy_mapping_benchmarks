#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tfMessage.h>
#include <tf2_ros/buffer.h>

#include <octomap/OcTree.h>
#include <octomap_ros/conversions.h>

#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <openvdb/openvdb.h>

#include <skimap/SkiMap.hpp>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include <chrono>
#include <string>

/// ScopeTicToc is a simple RAII class that keeps track of the duration of its lifetime,
/// and prints it during destruction.
class ScopeTicToc {
private:
    typedef std::chrono::high_resolution_clock clock;
    typedef std::chrono::microseconds res;
    clock::time_point t1, t2;
    std::string label;

public:
    explicit ScopeTicToc(const std::string &label = "default") :
            label(label) {
        t1 = clock::now();
    }

    ~ScopeTicToc() {
        t2 = clock::now();
        fmt::print("{},{}\n", label, std::chrono::duration_cast<res>(t2 - t1).count() / 1e6);
    }
};

/// timeit wraps a callable and prints its runtime when done.
template<typename F>
void timeit(const std::string &label, const F &&fn) {
    ScopeTicToc t(label);
    fn();
}

std::unique_ptr<octomap::OcTree> octomapCreateOctree(const float voxelSize = 0.05) {
    auto octree = std::make_unique<octomap::OcTree>(voxelSize);
    octree->setProbHit(0.7);
    octree->setProbMiss(0.4);
    octree->setClampingThresMin(0.12);
    octree->setClampingThresMax(0.97);

    return octree;
}

/// octomapBuildPlain integrates a vector of clouds into an octree.
void octomapBuildMapPlain(
        std::vector<sensor_msgs::PointCloud2ConstPtr> &clouds,
        std::vector<geometry_msgs::TransformStamped> &tfs,
        octomap::OcTree *octree) {

    typedef pcl::PointXYZ PCLPoint;
    typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;

    for (int i = 0; i < tfs.size(); i++) {
        PCLPointCloud pc;
        pcl::fromROSMsg(*clouds[i], pc);

        // geometry_msgs::TransformStamped to tf::StampedTransform
        tf::StampedTransform sensorToWorldTf;
        tf::transformStampedMsgToTF(tfs[i], sensorToWorldTf);

        // tf::StampedTransform to affine transformation matrix
        Eigen::Matrix4f sensorToWorld;
        pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

        // transform cloud to map frame
        pcl::transformPointCloud(pc, pc, sensorToWorld);

        // Filtering passes, also removing NaNs
        pcl::PassThrough<PCLPoint> passX;
        passX.setFilterFieldName("x");
        passX.setFilterLimits(-5, 5);
        pcl::PassThrough<PCLPoint> passY;
        passY.setFilterFieldName("y");
        passY.setFilterLimits(-5, 5);
        pcl::PassThrough<PCLPoint> passZ;
        passZ.setFilterFieldName("z");
        passZ.setFilterLimits(-5, 5);
        passX.setInputCloud(pc.makeShared());
        passX.filter(pc);
        passY.setInputCloud(pc.makeShared());
        passY.filter(pc);
        passZ.setInputCloud(pc.makeShared());
        passZ.filter(pc);

        octomap::point3d sensorOrigin = octomap::pointTfToOctomap(
                sensorToWorldTf.getOrigin());

        for (const auto &pt : pc) {
            octomap::point3d point(pt.x, pt.y, pt.z);

            // Clip the point to a maximum range.
            auto endPoint = point;
            static const double maxRange = 5.0;
            if ((point - sensorOrigin).norm() > maxRange) {
                endPoint = sensorOrigin + (point - sensorOrigin).normalized() * maxRange;
            }

            octomap::OcTreeKey key;
            if (octree->coordToKeyChecked(endPoint, key)) {
                octree->updateNode(key, float(1.0));
            }
        }
    }
}

/// octomapBuildPlain integrates a vector of clouds into an octree using the insertPointCloud builtin.
void octomapBuildMapBulkScan(
        std::vector<sensor_msgs::PointCloud2ConstPtr> &clouds,
        std::vector<geometry_msgs::TransformStamped> &tfs,
        octomap::OcTree *octree) {

    for (int i = 0; i < clouds.size(); i++) {
        // geometry_msgs::TransformStamped to tf::StampedTransform
        tf::StampedTransform sensorToWorldTf;
        tf::transformStampedMsgToTF(tfs[i], sensorToWorldTf);

        octomap::Pointcloud ompc;
        octomap::pointCloud2ToOctomap(*clouds[i], ompc);

        octomap::point3d sensorOrigin;
        sensorOrigin.x() = 0;
        sensorOrigin.y() = 0;
        sensorOrigin.z() = 0;

        tf::Pose tfPose;
        tfPose.setOrigin(sensorToWorldTf.getOrigin());
        tfPose.setRotation(sensorToWorldTf.getRotation());

        octomap::pose6d sensorPose = octomap::poseTfToOctomap(tfPose);

        octree->insertPointCloud(ompc, sensorOrigin, sensorPose, 5);
    }
}

/// vdbBuildMap integrates a vector of clouds into a vdb grid.
void vdbBuildMap(
        std::vector<sensor_msgs::PointCloud2ConstPtr> &clouds,
        std::vector<geometry_msgs::TransformStamped> &tfs,
        openvdb::FloatGrid::Ptr& grid) {

    typedef pcl::PointXYZ PCLPoint;
    typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;

    for (int i = 0; i < clouds.size(); i++) {
        PCLPointCloud pc;
        pcl::fromROSMsg(*clouds[i], pc);

        // geometry_msgs::TransformStamped to tf::StampedTransform
        tf::StampedTransform sensorToWorldTf;
        tf::transformStampedMsgToTF(tfs[i], sensorToWorldTf);

        // tf::StampedTransform to affine transformation matrix
        Eigen::Matrix4f sensorToWorld;
        pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

        // transform cloud to map frame
        pcl::transformPointCloud(pc, pc, sensorToWorld);

        // Filtering passes, also removing NaNs
        pcl::PassThrough<PCLPoint> passX;
        passX.setFilterFieldName("x");
        passX.setFilterLimits(-5, 5);
        pcl::PassThrough<PCLPoint> passY;
        passY.setFilterFieldName("y");
        passY.setFilterLimits(-5, 5);
        pcl::PassThrough<PCLPoint> passZ;
        passZ.setFilterFieldName("z");
        passZ.setFilterLimits(-5, 5);
        passX.setInputCloud(pc.makeShared());
        passX.filter(pc);
        passY.setInputCloud(pc.makeShared());
        passY.filter(pc);
        passZ.setInputCloud(pc.makeShared());
        passZ.filter(pc);

        octomap::point3d sensorOrigin = octomap::pointTfToOctomap(
                sensorToWorldTf.getOrigin());

        auto acc = grid->getAccessor();

        for (const auto & pt : pc) {
            octomap::point3d point(pt.x, pt.y, pt.z);

            // Clip the point to a maximum range.
            auto endPoint = point;
            static const double maxRange = 5.0;
            if ((point - sensorOrigin).norm() > maxRange) {
                endPoint = sensorOrigin + (point - sensorOrigin).normalized() * maxRange;
            }

            const auto idx = grid->worldToIndex(openvdb::Vec3d{endPoint.x(), endPoint.y(), endPoint.z()});
            const auto coord = openvdb::Coord(idx[0], idx[1], idx[2]);

            // The voxel value is hardcoded to 1.0: this is where you would put the measurement model
            // and recursive Bayes filter.
            acc.setValueOn(coord, 1.0);
        }
    }
}

using SkiCoordType = float;
using SkiIndexType = int16_t;
using SkiWeightType = uint16_t;

// A skimap voxel type with just a weight that can be used for recursive binary Bayes filtering.
struct SkiBinaryVoxel {
    float w;

    SkiBinaryVoxel() : w(0) {}

    explicit SkiBinaryVoxel(float w) : w(w) {}

    // Needed by the library
    SkiBinaryVoxel(SkiBinaryVoxel *other) {
        w = other->w;
    }

    SkiBinaryVoxel operator+(const SkiBinaryVoxel &rhs) const {
        SkiBinaryVoxel out;
        out.w = (w == 1) || (rhs.w == 1);
        return out;
    }

    friend std::ostream &operator<<(std::ostream &os, const SkiBinaryVoxel &voxel) {
        os << std::setprecision(std::numeric_limits<double>::digits10 + 2);
        os << double(voxel.w);
        return os;
    }

    friend std::istream &operator>>(std::istream &is, SkiBinaryVoxel &voxel) {
        double w;
        is >> w;
        voxel.w = w;
        return is;
    }
};

using MySkiMap = skimap::SkiMap<SkiBinaryVoxel, SkiIndexType, SkiCoordType>;

void skimapBuildMap(
        std::vector<sensor_msgs::PointCloud2ConstPtr> &clouds,
        std::vector<geometry_msgs::TransformStamped> &tfs,
        MySkiMap *skm) {

    typedef pcl::PointXYZ PCLPoint;
    typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;

    for (int i = 0; i < clouds.size(); i++) {
        PCLPointCloud pc;
        pcl::fromROSMsg(*clouds[i], pc);

        // geometry_msgs::TransformStamped to tf::StampedTransform
        tf::StampedTransform sensorToWorldTf;
        tf::transformStampedMsgToTF(tfs[i], sensorToWorldTf);

        // tf::StampedTransform to affine transformation matrix
        Eigen::Matrix4f sensorToWorld;
        pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

        // transform cloud to map frame
        pcl::transformPointCloud(pc, pc, sensorToWorld);

        // Filtering passes, also removing NaNs
        pcl::PassThrough<PCLPoint> passX;
        passX.setFilterFieldName("x");
        passX.setFilterLimits(-5, 5);
        pcl::PassThrough<PCLPoint> passY;
        passY.setFilterFieldName("y");
        passY.setFilterLimits(-5, 5);
        pcl::PassThrough<PCLPoint> passZ;
        passZ.setFilterFieldName("z");
        passZ.setFilterLimits(-5, 5);
        passX.setInputCloud(pc.makeShared());
        passX.filter(pc);
        passY.setInputCloud(pc.makeShared());
        passY.filter(pc);
        passZ.setInputCloud(pc.makeShared());
        passZ.filter(pc);

        octomap::point3d sensorOrigin = octomap::pointTfToOctomap(
                sensorToWorldTf.getOrigin());

        for (const auto &pt : pc) {
            octomap::point3d point(pt.x, pt.y, pt.z);

            // Clip the point to a maximum distance from the camera.
            auto endPoint = point;
            static const double maxRange = 5.0;
            if ((point - sensorOrigin).norm() > maxRange) {
                endPoint = sensorOrigin + (point - sensorOrigin).normalized() * maxRange;
            }

            // The voxel value is hardcoded to 1.0: this is where you would put the measurement model
            // and recursive Bayes filter.
            SkiBinaryVoxel vxl(1);

            skm->integrateVoxel(endPoint.x(), endPoint.y(), endPoint.z(), &vxl);
        }
    }
}


int main(int argc, char **argv) {
    if (argc < 1) {
        fmt::print("pass bag as argument");
        exit(1);
    }

    const std::string bagPath(argv[1]);
    rosbag::Bag bagIn;
    bagIn.open(bagPath, rosbag::BagMode::Read);

    rosbag::View view(bagIn);

    std::vector<sensor_msgs::PointCloud2ConstPtr> clouds;
    std::vector<geometry_msgs::TransformStamped> tfs;

    ros::Duration tfCacheTime(3600, 0);
    tf2_ros::Buffer tfBuffer(tfCacheTime);

    for (auto it = view.begin(); it != view.end(); it++) {
        const auto &topic = it->getTopic();
        if (topic == "/camera/depth/points") {
            auto msg = it->instantiate<sensor_msgs::PointCloud2>();
            if (!msg) {
                continue;
            }
            clouds.emplace_back(msg);
        } else if (topic == "/tf") {
            tf::tfMessageConstPtr msg = it->instantiate<tf::tfMessage>();
            if (!msg) {
                continue;
            }

            for (const auto &tsf : msg->transforms) {
                tfBuffer.setTransform(tsf, "bag", false);
            }
        }
    }

    if (clouds.size() == 0) {
        fmt::print("no clouds found - check the topic name.\n");
        exit(1);
    }

    std::sort(clouds.begin(), clouds.end(), [](const auto &lhs, const auto &rhs) {
        return lhs->header.stamp < rhs->header.stamp;
    });

    for (const auto &c : clouds) {
        geometry_msgs::TransformStamped sensorToWorldTf;
        try {
            sensorToWorldTf = tfBuffer.lookupTransform("world", c->header.frame_id.substr(1), c->header.stamp);
        } catch (tf2::ExtrapolationException &e) {
            break;
        }

        tfs.emplace_back(sensorToWorldTf);
    }

    if (tfs.empty()) {
        fmt::print("no transforms found.\n");
        exit(1);
    }

    clouds.resize(tfs.size());

    {
        auto skm = std::make_unique<MySkiMap>(0.05);

        timeit("SkiMap", [&]() {
            skimapBuildMap(clouds, tfs, skm.get());
        });

        // Get the voxels out and save them to an octomap for visualization.
        std::vector<MySkiMap::Voxel3D> voxels;
        skm->fetchVoxels(voxels);

        fmt::print(std::cerr, "skimap: created {} voxels\n", voxels.size());

        auto octree = octomapCreateOctree(0.051);
        for (const auto & vxl : voxels) {
            octomap::point3d worldPoint(vxl.x, vxl.y, vxl.z);
            octomap::OcTreeKey key;
            octree->coordToKeyChecked(worldPoint, key);
            octree->updateNode(key, true);
        }
        octree->writeBinary("mytree_skimap.bt");
    }

    {
        auto octree = octomapCreateOctree(0.05);

        timeit("OctoMap", [&]() {
            octomapBuildMapPlain(clouds, tfs, octree.get());
        });

        octree->writeBinary("mytree.bt");
    }

//    {
//        auto octree = octomapCreateOctree();
//
//        timeit("octomap_bulk_build", [&]() {
//            octomapBuildMapBulkScan(clouds, tfs, octree.get());
//        });
//    }

    {
        openvdb::initialize();

        auto grid = openvdb::FloatGrid::create(0.0);
        grid->setTransform(
                openvdb::math::Transform::createLinearTransform(/*voxel size=*/0.05));
        grid->setGridClass(openvdb::GRID_LEVEL_SET);

        timeit("OpenVDB", [&]() {
            vdbBuildMap(clouds, tfs, grid);
        });

        // Create a VDB file object.
        openvdb::io::File file("mygrids.vdb");

        // Add the grid pointer to a container.
        openvdb::GridPtrVec grids;
        grids.push_back(grid);

        // Write out the contents of the container.
        file.write(grids);
        file.close();

        //
        // create an octomap with the same data for easy visualization.

        auto octree = octomapCreateOctree(0.051);

        for (auto it = grid->cbeginValueOn(); it; ++it) {
            auto idx = grid->indexToWorld(it.getCoord());
            octomap::point3d worldP(idx.x(), idx.y(), idx.z());

            octomap::OcTreeKey key;
            octree->coordToKeyChecked(worldP, key);

            octree->updateNode(key, true);
        }

        octree->writeBinary("mytree_vdb.bt");
    }
}
