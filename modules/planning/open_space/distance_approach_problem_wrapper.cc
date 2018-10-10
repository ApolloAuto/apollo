bool FrameOpenSpace::VPresentationObstacle() {
  obstacles_num_ = obstacles_.Items().size();
  if (obstacles_num_ == 0) {
    AINFO << "no obstacle by perception";
    return false;
  }
  obstacles_vertices_num_ = 4 * Eigen::MatrixXd::Ones(obstacles_num_, 1);
  for (const auto &obstacle : obstacles_.Items()) {
    Box2d obstacle_box = obstacle->PerceptionBoundingBox();
    std::vector<Vec2d> vertices_ccw = obstacle_box.GetAllCorners();
    std::vector<Vec2d> vertices_cw;
    while (!vertices_ccw.empty()) {
      vertices_cw.emplace_back(vertices_ccw.back());
      vertices_ccw.pop_back();
    }
    // As the obstacle is a closed convex set, the first vertice is repeated at
    // the end of the vector to help transform all four edges to inequality
    // constraint
    vertices_cw.push_back(vertices_cw.front());
    obstacles_vertices_vec_.emplace_back(vertices_cw);
  }
  return true;
}

bool FrameOpenSpace::HPresentationObstacle() {
  obstacles_A_ = Eigen::MatrixXd::Zero(obstacles_vertices_num_.sum(), 2);
  obstacles_b_ = Eigen::MatrixXd::Zero(obstacles_vertices_num_.sum(), 1);
  // vertices using H-represetntation
  if (!ObsHRep(obstacles_num_, obstacles_vertices_num_, obstacles_vertices_vec_,
               &obstacles_A_, &obstacles_b_)) {
    AINFO << "Fail to present obstacle in hyperplane";
    return false;
  }
  return true;
}

bool FrameOpenSpace::ObsHRep(
    const std::size_t &obstacles_num,
    const Eigen::MatrixXd &obstacles_vertices_num,
    const std::vector<std::vector<Vec2d>> &obstacles_vertices_vec,
    Eigen::MatrixXd *A_all, Eigen::MatrixXd *b_all) {
  if (obstacles_num != obstacles_vertices_vec.size()) {
    AINFO << "obstacles_num != obstacles_vertices_vec.size()";
    return false;
  }

  A_all->resize(obstacles_vertices_num.sum(), 2);
  b_all->resize(obstacles_vertices_num.sum(), 1);

  int counter = 0;
  double kEpsilon = 1.0e-5;
  // start building H representation
  for (std::size_t i = 0; i < obstacles_num; ++i) {
    std::size_t current_vertice_num = obstacles_vertices_num(i, 0);
    Eigen::MatrixXd A_i(current_vertice_num, 2);
    Eigen::MatrixXd b_i(current_vertice_num, 1);

    // take two subsequent vertices, and computer hyperplane
    for (std::size_t j = 0; j < current_vertice_num; ++j) {
      Vec2d v1 = obstacles_vertices_vec[i][j];
      Vec2d v2 = obstacles_vertices_vec[i][j + 1];

      Eigen::MatrixXd A_tmp(2, 1), b_tmp(1, 1), ab(2, 1);
      // find hyperplane passing through v1 and v2
      if (std::abs(v1.x() - v2.x()) < kEpsilon) {
        if (v2.y() < v1.y()) {
          A_tmp << 1, 0;
          b_tmp << v1.x();
        } else {
          A_tmp << -1, 0;
          b_tmp << -v1.x();
        }
      } else if (std::abs(v1.y() - v2.y()) < kEpsilon) {
        if (v1.x() < v2.x()) {
          A_tmp << 0, 1;
          b_tmp << v1.y();
        } else {
          A_tmp << 0, -1;
          b_tmp << -v1.y();
        }
      } else {
        Eigen::MatrixXd tmp1(2, 2);
        tmp1 << v1.x(), 1, v2.x(), 1;
        Eigen::MatrixXd tmp2(2, 1);
        tmp2 << v1.y(), v2.y();
        ab = tmp1.inverse() * tmp2;
        double a = ab(0, 0);
        double b = ab(1, 0);

        if (v1.x() < v2.x()) {
          A_tmp << -a, 1;
          b_tmp << b;
        } else {
          A_tmp << a, -1;
          b_tmp << -b;
        }
      }

      // store vertices
      A_i.block(j, 0, 1, 2) = A_tmp.transpose();
      b_i.block(j, 0, 1, 1) = b_tmp;
    }

    A_all->block(counter, 0, A_i.rows(), 2) = A_i;
    b_all->block(counter, 0, b_i.rows(), 1) = b_i;
    counter += current_vertice_num;
  }
  return true;
}