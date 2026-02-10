#ifndef SCAN_OVERLAP_TYPES_H_
#define SCAN_OVERLAP_TYPES_H_

#include <eigen3/Eigen/Dense>
#include <map>
#include <string>
#include <vector>

#define PRINT_DIM(X) std::cout << #X << " rows " << X.rows() << " cols " << X.cols() << std::endl;
#define RAD2DEG(X) (180.0 / M_PI * (X))

namespace scan_overlap
{
    using Vector3 = Eigen::Vector3d;
    using Vector2 = Eigen::Vector2d;
    using VectorVector3 = std::vector<Vector3>;
    using VectorVector2 = std::vector<Vector2>;
    using Quaternion = Eigen::Quaterniond;
    using Transform3 = Eigen::Affine3d;
    using Transform2 = Eigen::Affine2d;
    using VectorTransform3 =
        std::vector<Transform3>;
    using VectorTransform2 =
        std::vector<Transform2>;
    using Scan = std::vector<double>;
    using LaserSpecs = std::map<std::string, std::string>;
    using Cloud = std::vector<Vector2>;

    struct Node
    {
        int id;
        scan_overlap::Cloud cloud;
        std::vector<int> adj;
        std::vector<double> coeffs;

        Node(int id_) : id(id_), cloud(), adj() {};
        
        Node(int id_, std::vector<int> adj_) : id(id_), cloud(), adj(adj_) {};

        Node(int id_, scan_overlap::Cloud cloud_) : id(id_), cloud(cloud_), adj() {}

        void setCloud(const scan_overlap::Cloud& cloud_){
            cloud = cloud_;
        }

        void setCoeffs(const std::vector<double> coeffs_){
            coeffs = coeffs_;
        }
    };
    using Graph = std::vector<Node>;

    struct ErrorData
    {
        int firstIdx;
        int lastIdx;
        double length;
        double errTransl;
        double errRot;
    };
    using VectorErrorData = std::vector<ErrorData>;

    struct Edge
    {
        int src;
        int dst;

        Edge(int src_, int dst_) : src(src_), dst(dst_) {}
    };

} // namespace scan_overlap

#endif // SCAN_OVERLAP_TYPES_H_