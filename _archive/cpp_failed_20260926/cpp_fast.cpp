// cpp_fast.cpp — C ABI, вызывается из Python через ctypes (GIL auto-release)
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

extern "C" {

long rotate_roi_c(const float* pts, long n,
                  float fwd_min, float fwd_max,
                  float lat_lim, float z_min, float z_max,
                  float* out) {
    long m = 0;
    for (long i = 0; i < n; i++) {
        float x = -pts[i*3 + 1];
        float y = pts[i*3 + 0];
        float z = pts[i*3 + 2];
        if (x < fwd_min || x > fwd_max) continue;
        if (y < -lat_lim || y > lat_lim) continue;
        if (z < z_min || z > z_max) continue;
        out[m*3 + 0] = x;
        out[m*3 + 1] = y;
        out[m*3 + 2] = z;
        m++;
    }
    return m;
}

long voxel_hash_c(const float* pts, long n, float vox, long* out) {
    if (n == 0) return 0;
    std::vector<std::pair<int64_t, int64_t>> kv(n);
    float inv = 1.0f / vox;
    const int64_t OFFSET = 1 << 20;
    for (long i = 0; i < n; i++) {
        int64_t kx = (int64_t)std::floor(pts[i*3 + 0] * inv) + OFFSET;
        int64_t ky = (int64_t)std::floor(pts[i*3 + 1] * inv) + OFFSET;
        int64_t kz = (int64_t)std::floor(pts[i*3 + 2] * inv) + OFFSET;
        int64_t key = (kx << 42) | (ky << 21) | kz;
        kv[i] = {key, (int64_t)i};
    }
    std::sort(kv.begin(), kv.end());
    long m = 0;
    int64_t last = std::numeric_limits<int64_t>::min();
    for (auto& p : kv) {
        if (p.first != last) {
            out[m++] = p.second;
            last = p.first;
        }
    }
    std::sort(out, out + m);
    return m;
}

long grid_dbscan_c(const float* pts, long n,
                   float eps, int min_pts, int* out) {
    if (n < (long)min_pts) {
        for (long i = 0; i < n; i++) out[i] = -1;
        return n;
    }
    float inv = 1.0f / eps;
    const int64_t OFFSET = 1 << 20;

    std::vector<std::pair<int64_t, int32_t>> sorted(n);
    for (long i = 0; i < n; i++) {
        int64_t kx = (int64_t)std::floor(pts[i*3 + 0] * inv) + OFFSET;
        int64_t ky = (int64_t)std::floor(pts[i*3 + 1] * inv) + OFFSET;
        int64_t kz = (int64_t)std::floor(pts[i*3 + 2] * inv) + OFFSET;
        int64_t key = (kx << 42) | (ky << 21) | kz;
        sorted[i] = {key, (int32_t)i};
    }
    std::sort(sorted.begin(), sorted.end());

    std::vector<int32_t> parent(n);
    for (long i = 0; i < n; i++) parent[i] = (int32_t)i;

    auto find = [&](int32_t x) -> int32_t {
        int32_t root = x;
        while (parent[root] != root) root = parent[root];
        while (parent[x] != root) {
            int32_t nxt = parent[x];
            parent[x] = root;
            x = nxt;
        }
        return root;
    };

    float eps_sq = eps * eps;

    for (long j = 0; j < n; j++) {
        int64_t k = sorted[j].first;
        int32_t idx_j = sorted[j].second;
        int64_t cx = (k >> 42) & 0x1FFFFF;
        int64_t cy = (k >> 21) & 0x1FFFFF;
        int64_t cz = k & 0x1FFFFF;
        float px = pts[idx_j*3 + 0];
        float py = pts[idx_j*3 + 1];
        float pz = pts[idx_j*3 + 2];
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                for (int dz = -1; dz <= 1; dz++) {
                    int64_t nk = ((cx+dx) << 42) | ((cy+dy) << 21) | (cz+dz);
                    auto lo = std::lower_bound(
                        sorted.begin(), sorted.end(),
                        std::make_pair(nk, std::numeric_limits<int32_t>::min()));
                    auto hi = std::upper_bound(
                        sorted.begin(), sorted.end(),
                        std::make_pair(nk, std::numeric_limits<int32_t>::max()));
                    for (auto it = lo; it != hi; ++it) {
                        int32_t k2 = it->second;
                        if (k2 <= idx_j) continue;
                        float ddx = px - pts[k2*3 + 0];
                        float ddy = py - pts[k2*3 + 1];
                        float ddz = pz - pts[k2*3 + 2];
                        if (ddx*ddx + ddy*ddy + ddz*ddz <= eps_sq) {
                            int32_t ra = find(idx_j);
                            int32_t rb = find(k2);
                            if (ra != rb) parent[ra] = rb;
                        }
                    }
                }
            }
        }
    }
    std::vector<int32_t> labels(n);
    for (long i = 0; i < n; i++) labels[i] = find((int32_t)i);

    std::vector<std::pair<int32_t, int32_t>> cnt(n);
    for (long i = 0; i < n; i++) cnt[i] = {labels[i], i};
    std::sort(cnt.begin(), cnt.end());

    std::vector<int32_t> uniq_labels, uniq_counts;
    int32_t prev = INT32_MIN;
    int32_t c = 0;
    for (long i = 0; i < n; i++) {
        if (cnt[i].first != prev) {
            if (prev != INT32_MIN) {
                uniq_labels.push_back(prev);
                uniq_counts.push_back(c);
            }
            prev = cnt[i].first;
            c = 1;
        } else c++;
    }
    if (prev != INT32_MIN) {
        uniq_labels.push_back(prev);
        uniq_counts.push_back(c);
    }

    std::vector<int32_t> remap;
    remap.resize((size_t)(*std::max_element(labels.begin(), labels.end()) + 1), -1);
    int32_t next_lbl = 0;
    for (size_t i = 0; i < uniq_labels.size(); i++) {
        if (uniq_counts[i] >= min_pts) remap[uniq_labels[i]] = next_lbl++;
    }
    for (long i = 0; i < n; i++) out[i] = remap[labels[i]];
    return n;
}

}  // extern "C"