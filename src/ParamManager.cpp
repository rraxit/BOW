//
// Created by redwan on 11/10/24.
//

#include "bow/ParamManager.h"

bool Matrix2D::operator==(const Matrix2D& other) const {
    return other.data.size() == data.size() && other.data[0].size() == data[0].size();
}

std::ostream& operator<<(std::ostream& os, const Matrix2D& d) {
    for (const auto& row : d.data) {
        for (float value : row) {
            os << value << " ";
        }
        os << "\n";
    }
    return os;
}

YAML::Node YAML::convert<Matrix2D>::encode(const Matrix2D& rhs) {
    Node node;
    for (const auto& d : rhs.data) {
        node.push_back(d);
    }
    return node;
}

bool YAML::convert<Matrix2D>::decode(const Node& node, Matrix2D& rhs) {
    for (const auto& temp : node) {
        std::vector<float> cols;
        for (const auto& item : temp) {
            cols.push_back(item.as<float>());
        }
        rhs.data.push_back(cols);
    }
    return true;
}

param_manager::param_manager(const std::string& file) {
    config_ = YAML::LoadFile(file);
}

std::shared_ptr<param_manager> param_manager::getSharedPtr() {
    return shared_from_this();
}

std::vector<Triangle2D> param_manager::get_triangles() {
    return get_triangles("triangles");
}

std::vector<Triangle2D> param_manager::get_triangles(const std::string& field) {
    std::vector<Triangle2D> triangles;

    try {
        if (!config_[field]) {
            throw std::runtime_error("Field '" + field + "' not found in YAML configuration");
        }

        const YAML::Node& triangles_node = config_[field];

        if (!triangles_node.IsSequence()) {
            throw std::runtime_error("Field '" + field + "' is not a sequence");
        }

        triangles.reserve(triangles_node.size());

        for (size_t i = 0; i < triangles_node.size(); ++i) {
            try {
                Triangle2D triangle = triangles_node[i].as<Triangle2D>();
                triangles.push_back(triangle);
            } catch (const YAML::Exception& e) {
                std::cerr << "Warning: Failed to parse triangle " << i << ": " << e.what() << std::endl;
                continue; // Skip invalid triangles
            }
        }

        std::cout << "Successfully loaded " << triangles.size() << " triangles from field '" << field << "'" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error reading triangles: " << e.what() << std::endl;
        throw;
    }

    return triangles;
}