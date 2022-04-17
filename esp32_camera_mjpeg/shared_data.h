extern long LAST_REQUEST_TIMESTAMP;
extern int LAST_DISTANCE;
extern int CURRENT_DISTANCE;
extern bool IS_TURNING;

struct control_data {
  int frame_width, frame_height;
  int laser_left, laser_top;
  int center_width;
  int center_left_line, center_right_line;
  int landmine_left, landmine_top;
  int landmine_stop_distance;
};
