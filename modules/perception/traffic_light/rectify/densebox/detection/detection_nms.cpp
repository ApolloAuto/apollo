//
// Created by gaohan02 on 16-8-2.
//
#include "module/perception/traffic_light/rectify/densebox/detection/detection_nms.hpp"

namespace adu {
namespace perception {
namespace traffic_light {
using std::min;
using std::max;
bool
nms(const std::vector<float> &col, const std::vector<float> &row, const std::vector<float> &size_w,
    const std::vector<float> &size_h, const std::vector<float> &score, const int height,
    const int width,
    const float overlappingThreshold, std::vector<float> &col_nms, std::vector<float> &row_nms,
    std::vector<float> &size_w_nms, std::vector<float> &size_h_nms, std::vector<float> &score_nms) {
  col_nms.clear();
  row_nms.clear();
  size_w_nms.clear();
  size_h_nms.clear();
  score_nms.clear();

  if (overlappingThreshold > 1.0 || overlappingThreshold < 0.0) {
    return false;
  }
  if (row.empty()) {
    return true;
  }
  int num_candidates = row.size();
  std::vector<int> parent(num_candidates);
  for (int i = 0; i < num_candidates; i++) {
    parent[i] = i;
  }
  std::vector<int> rank(num_candidates, 0);
  for (int i = 0; i < num_candidates; i++) {
    for (int j = i + 1; j < num_candidates; j++) {
      float h = min(row[i] + size_h[i], row[j] + size_h[j]) - max(row[i], row[j]);
      float w = min(col[i] + size_w[i], col[j] + size_w[j]) - max(col[i], col[j]);
      float s = max(h, float(0.0)) * max(w, float(0.0));
      if (s / (size_w[i] * size_h[i] + size_w[j] * size_h[j] - s) >=
          overlappingThreshold) {
        //find parent of i and compress path
        int pi = i;
        while (parent[pi] != pi) {
          pi = parent[pi];
        }
        int q = i;
        while (parent[q] != q) {
          int t = q;
          q = parent[q];
          parent[t] = pi;
        }
        //find parent of j and compress path
        int pj = j;
        while (parent[pj] != pj) {
          pj = parent[pj];
        }
        q = j;
        while (parent[q] != q) {
          int t = q;
          q = parent[q];
          parent[t] = pj;
        }
        if (pi != pj) {
          if (rank[pj] < rank[pi]) {
            parent[pj] = pi;
          } else if (rank[pi] < rank[pj]) {
            parent[pi] = pj;
          } else {
            parent[pj] = pi;
            rank[pi]++;
          }
        }
      }
    }
  }
  std::vector<int> label(num_candidates, -1);
  int l = 0;
  for (int i = 0; i < num_candidates; i++) {
    if (parent[i] == i) {
      label[i] = l++;
    }
  }
  for (int i = 0; i < num_candidates; i++) {
    if (parent[i] == i) {
      continue;
    }
    int pi = i;
    while (pi != parent[pi]) {
      pi = parent[pi];
    }
    label[i] = label[pi];
  }
  std::vector<float> weight(num_candidates, 0.0);
  int num_groups = l;
  std::vector<float> weight_sum(num_groups, 0.0);
  for (int i = 0; i < num_candidates; i++) {
    weight[i] = log(1.0 + exp(score[i]));
    weight_sum[label[i]] += weight[i];
  }
  for (int i = 0; i < num_candidates; i++) {
    weight[i] /= weight_sum[label[i]];
  }
  std::vector<float> row_avg(num_groups, 0.0);
  std::vector<float> col_avg(num_groups, 0.0);
  std::vector<float> size_w_avg(num_groups, 0.0);
  std::vector<float> size_h_avg(num_groups, 0.0);
  for (int i = 0; i < num_candidates; i++) {
    size_w_avg[label[i]] += size_w[i] * weight[i];
  }
  for (int i = 0; i < num_candidates; i++) {
    size_h_avg[label[i]] += size_h[i] * weight[i];
  }
  for (int i = 0; i < num_candidates; i++) {
    row_avg[label[i]] += (row[i] + size_h[i] / 2) * weight[i];
    col_avg[label[i]] += (col[i] + size_w[i] / 2) * weight[i];
  }
  for (int i = 0; i < num_groups; i++) {
    row_avg[i] = row_avg[i] - size_h_avg[i] / 2;
    col_avg[i] = col_avg[i] - size_w_avg[i] / 2;
  }
  //remove embbed faces
  std::vector<std::vector<bool>> predicate(num_groups, std::vector<bool>(num_groups, false));
  for (int i = 0; i < num_groups; i++) {
    for (int j = i + 1; j < num_groups; j++) {
      float h = min(row_avg[i] + size_h_avg[i], row_avg[j] + size_h_avg[j]) -
          max(row_avg[i], row_avg[j]);
      float w = min(col_avg[i] + size_w_avg[i], col_avg[j] + size_w_avg[j]) -
          max(col_avg[i], col_avg[j]);
      float s = max(h, (float) 0.0) * max(w, (float) 0.0);
      if (s / (size_h_avg[i] * size_w_avg[i]) >= overlappingThreshold ||
          s / (size_h_avg[j] * size_w_avg[j]) >= overlappingThreshold) {
        predicate[i][j] = true;
        predicate[j][i] = true;
      }
    }
  }
  std::vector<bool> flag(num_groups, true);
  for (int i = 0; i < num_groups; i++) {
    for (int j = 0; j < num_groups; j++) {
      if (predicate[i][j] && weight_sum[j] > weight_sum[i]) {
        flag[i] = false;
      }
    }
  }
  //result
  for (int i = 0; i < num_groups; i++) {
    if (!flag[i]) {
      continue;
    }
    float r = row_avg[i];
    float c = col_avg[i];
    float w = size_w_avg[i];
    float h = size_h_avg[i];
    float score = weight_sum[i];

    if (w <= 0 || h <= 0 || c >= width || r >= height
        || c + w - 1 < 0 || r + h - 1 < 0) {
      continue;
    }
    if (r < 0) {
      h += r;
      r = 0;
    }
    if (c < 0) {
      w += c;
      c = 0;
    }
    if (r + h > height) {
      h = height - r;
    }
    if (c + w > width) {
      w = width - c;
    }

    row_nms.push_back(r);
    col_nms.push_back(c);
    size_w_nms.push_back(w);
    size_h_nms.push_back(h);
    score_nms.push_back(score);
  }
  return true;
}

//template <typename Dtype>
bool nms(const std::vector<BoundBox_t> &bbxes, const int img_height, const int img_width,
         const float overlappingThreshold, std::vector<BoundBox_t> &bbxes_nms) {
  bbxes_nms.clear();
  if (bbxes.empty()) {
    return true;
  }
  std::vector<float> col, row, size_w, size_h, score;
  std::vector<float> col_nms, row_nms, size_w_nms, size_h_nms, score_nms;
  for (int i = 0; i < bbxes.size(); i++) {
    int x = bbxes[i].rect.x;
    int y = bbxes[i].rect.y;
    int w = bbxes[i].rect.width;
    int h = bbxes[i].rect.height;
    if (w <= 0 || h <= 0 || x >= img_width || y >= img_height
        || x + w - 1 < 0 || y + h - 1 < 0) {
      continue;
    }
    if (x < 0) {
      w += x;
      x = 0;
    }
    if (y < 0) {
      h += y;
      y = 0;
    }
    if (x + w > img_width) {
      w = img_width - x;
    }
    if (y + h > img_height) {
      h = img_height - y;
    }

    col.push_back(x);
    row.push_back(y);
    size_w.push_back(w);
    size_h.push_back(h);
    score.push_back(bbxes[i].score);
  }
  if (!nms(col, row, size_w, size_h, score, img_height, img_width, overlappingThreshold,
           col_nms,
           row_nms, size_w_nms, size_h_nms, score_nms)) {
    bbxes_nms.clear();
    return false;
  }
  //Dtype id = bbxes[0].id;
  for (int i = 0; i < col_nms.size(); i++) {
    int x = (int) col_nms[i];
    int y = (int) row_nms[i];
    int w = (int) size_w_nms[i];
    int h = (int) size_h_nms[i];
    if (w <= 0 || h <= 0 || x >= img_width || y >= img_height
        || x + w - 1 < 0 || y + h - 1 < 0) {
      continue;
    }
    if (x < 0) {
      w += x;
      x = 0;
    }
    if (y < 0) {
      h += y;
      y = 0;
    }
    if (x + w > img_width) {
      w = img_width - x;
    }
    if (y + h > img_height) {
      h = img_height - y;
    }
    if (w <= 0 || h <= 0
        || x < 0 || y < 0
        || x >= img_width || y >= img_height
        || x + w - 1 >= img_width || y + h - 1 >= img_height
        || x + w - 1 < 0 || y + h - 1 < 0) {
      continue;
    }

    BoundBox_t bbx;
    bbx.rect.x = x;
    bbx.rect.y = y;
    bbx.rect.width = w;
    bbx.rect.height = h;
    bbx.score = score_nms[i];
    //  bbx.id=id;
    bbxes_nms.push_back(bbx);
  }
  return true;
}
}
}
}
