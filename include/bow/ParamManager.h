//
// Created by redwan on 11/10/24.
//

#ifndef BOW_PARAMMANAGER_H
#define BOW_PARAMMANAGER_H



#include <yaml-cpp/yaml.h>
#include <memory>
#include <vector>
#include <string>

struct Matrix2D {
    std::vector<std::vector<float>> data;
    bool operator==(const Matrix2D& other) const;
    friend std::ostream& operator<<(std::ostream& os, const Matrix2D& d);
};

// Triangle structure to hold 3 vertices with x,y coordinates
struct Triangle2D {
    std::array<std::array<float, 2>, 3> vertices; // 3 vertices, each with x,y coordinates

    Triangle2D() = default;
    Triangle2D(const std::array<std::array<float, 2>, 3>& verts) : vertices(verts) {}

    bool operator==(const Triangle2D& other) const {
        return vertices == other.vertices;
    }

    friend std::ostream& operator<<(std::ostream& os, const Triangle2D& triangle) {
        os << "Triangle: [";
        for (size_t i = 0; i < 3; ++i) {
            os << "[" << triangle.vertices[i][0] << ", " << triangle.vertices[i][1] << "]";
            if (i < 2) os << ", ";
        }
        os << "]";
        return os;
    }
};

namespace YAML {
    template<>
    struct convert<Matrix2D> {
        static Node encode(const Matrix2D& rhs);
        static bool decode(const Node& node, Matrix2D& rhs);
    };

    template<>
    struct convert<Triangle2D> {
        static Node encode(const Triangle2D& rhs) {
            Node node;
            for (size_t i = 0; i < 3; ++i) {
                Node vertex;
                vertex.push_back(rhs.vertices[i][0]);
                vertex.push_back(rhs.vertices[i][1]);
                node.push_back(vertex);
            }
            return node;
        }

        static bool decode(const Node& node, Triangle2D& rhs) {
            if (!node.IsSequence() || node.size() != 3) {
                return false;
            }

            try {
                for (size_t i = 0; i < 3; ++i) {
                    if (!node[i].IsSequence() || node[i].size() != 2) {
                        return false;
                    }
                    rhs.vertices[i][0] = node[i][0].as<float>();
                    rhs.vertices[i][1] = node[i][1].as<float>();
                }
                return true;
            } catch (const YAML::Exception&) {
                return false;
            }
        }
    };
}

class param_manager : public std::enable_shared_from_this<param_manager> {
public:
    explicit param_manager(const std::string& file);

    template<class T>
    T get_param(const std::string& field);

    template<class T>
    T get_param(const std::string& field1, const std::string& field2);

    template<class T>
    T get_param(const std::string& field1, const std::string& field2, const std::string& field3);

    template<class T>
    void get_obstacles(T& result);

    // Method to read triangles from YAML
    std::vector<Triangle2D> get_triangles();

    // Alternative method with field specification
    std::vector<Triangle2D> get_triangles(const std::string& field);

    std::shared_ptr<param_manager> getSharedPtr();

private:
    YAML::Node config_;
};

using ParamPtr = std::shared_ptr<param_manager>;

#include "param_manager.tpp"



#endif //BOW_PARAMMANAGER_H
