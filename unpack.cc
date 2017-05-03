#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <cassert>
#include <vector>

static const int SIZE_BLOCK = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

static const int PACKET_SIZE = 1206;
static const int BLOCKS_PER_PACKET = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

static const float ROTATION_RESOLUTION = 0.01f; /**< degrees */
static const uint16_t ROTATION_MAX_UNITS = 36000; /**< hundredths of degrees */

static const float DISTANCE_MAX = 130.0f;        /**< meters */
static const float DISTANCE_RESOLUTION = 0.002f; /**< meters */
static const float DISTANCE_MAX_UNITS = (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0);

static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

static const int    VLP16_FIRINGS_PER_BLOCK =   2;
static const int    VLP16_SCANS_PER_FIRING  =  16;
static const float  VLP16_BLOCK_TDURATION   = 110.592f;   // [µs]
static const float  VLP16_DSR_TOFFSET       =   2.304f;   // [µs]
static const float  VLP16_FIRING_TOFFSET    =  55.296f;   // [µs]

static const double PI = acos(-1.0);

const int LASER_ANGLES[] = {-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15};

struct Point {
  double x, y, z, ts;
  int intensity;
  Point() {}
  Point(double _x, double _y, double _z, double _ts, int _in): x(_x), y(_y), z(_z), ts(_ts), intensity(_in) {}
};

union two_bytes {
  uint16_t uint;
  uint8_t bytes[2];
};

struct raw_block_t {
  uint16_t header;        ///< UPPER_BANK or LOWER_BANK
  uint16_t azimuth;      ///< 0-35999, divide by 100 to get degrees
  uint8_t  data[BLOCK_DATA_SIZE];
};

struct raw_packet_t {
  raw_block_t blocks[BLOCKS_PER_PACKET];
  uint32_t timestamp;
  uint16_t factory;
};

std::vector<Point> pts;
int scan_index;
int prev_azimuth;

void save_pts(char *filepath) {
  printf("%s\n", filepath);
  FILE *fp = fopen(filepath, "w");
  for (size_t i = 0; i < pts.size(); ++i) {
    fprintf(fp, "%.10f,%.10f,%.10f,%d,%.6f\n", pts[i].x, pts[i].y, pts[i].z, pts[i].intensity, pts[i].ts);
  }
  fclose(fp);
}

Point calc_point(double distance, int azimuth, int laser_id, double timestamp, int intensity) {
  distance *= DISTANCE_RESOLUTION;
  double omega = LASER_ANGLES[laser_id] * PI / 180.0;
  double alpha = azimuth * ROTATION_RESOLUTION * PI / 180.0;
  double X = distance * cos(omega) * sin(alpha);
  double Y = distance * cos(omega) * cos(alpha);
  double Z = distance * sin(omega);
  return Point(X, Y, Z, timestamp, intensity);
}

uint8_t rawdata[3000];

void unpack(char *dir, char *filename) {
  FILE *fp = fopen(filename, "rb");
  while (fread(rawdata, 1, 1223, fp) == 1223) {
    char ts[20]; ts[17] = 0;
    for (int i = 0; i < 17; ++i) ts[i] = rawdata[i];
    const raw_packet_t *raw = (const raw_packet_t*)&rawdata[17];
    double timestamp = atof(ts) - 1327.104 / 1000000;
    assert(raw->factory == 0x2237);
    int azimuth_diff = 0;
    int last_azimuth_diff = 0;
    for (int block = 0; block < BLOCKS_PER_PACKET; ++block) {
      assert(UPPER_BANK == raw->blocks[block].header);
      int azimuth = raw->blocks[block].azimuth;
      if (block + 1 < BLOCKS_PER_PACKET) {
        azimuth_diff = (36000 + raw->blocks[block + 1].azimuth - azimuth) % 36000;
        last_azimuth_diff = azimuth_diff;
      } else {
        azimuth_diff = last_azimuth_diff;
      }
      for (int firing = 0, k = 0; firing < VLP16_FIRINGS_PER_BLOCK; ++firing) {
        if (prev_azimuth != -1 && azimuth < prev_azimuth && !pts.empty()) {
          static char filepath[200];
          sprintf(filepath, "%s/i%08d_%.6f.csv", dir, scan_index, pts[0].ts);
          save_pts(filepath);
          scan_index += 1;
          pts.clear();
        }
        int seq_index = block * 2 + firing;
        for (int dsr = 0; dsr < VLP16_SCANS_PER_FIRING; ++dsr, k += RAW_SCAN_SIZE) {
          union two_bytes tmp;
          tmp.bytes[0] = raw->blocks[block].data[k];
          tmp.bytes[1] = raw->blocks[block].data[k + 1];
          int intensity = raw->blocks[block].data[k + 2];
          double time_offset = (seq_index * VLP16_FIRING_TOFFSET + dsr * VLP16_DSR_TOFFSET) / 1000000.0;
          if (tmp.uint != 0) {
            pts.push_back(calc_point(tmp.uint, azimuth, dsr, timestamp + time_offset, intensity));
          }
        }
        prev_azimuth = azimuth;
        azimuth += azimuth_diff / 2;
        azimuth %= ROTATION_MAX_UNITS;
      }
    }
  }
}

int main(int argc, char **argv) {
  if (argc == 1) {
    puts("./unpack <destination directory> <file1> <file2> ...");
    return 0;
  }
  scan_index = 0;
  prev_azimuth = -1;
  pts.clear();
  for (int i = 2; i < argc; ++i) {
    unpack(argv[1], argv[2]);
  }
  return 0;
}
