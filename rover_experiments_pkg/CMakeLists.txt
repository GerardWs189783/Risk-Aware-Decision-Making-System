cmake_minimum_required(VERSION 3.8)
project(rover_experiments_pkg)

# --- 1. FIND DEPENDENCIES ---
find_package(ament_cmake REQUIRED)

# --- 2. INSTALL PYTHON SCRIPTS ---
install(PROGRAMS
  scripts/metrics_logger_e1.py
  scripts/metrics_logger_e2.py
  scripts/deep_dive_logger.py
  scripts/e2_deep_dive_logger.py
  scripts/e3_logger.py
  scripts/metrics_logger_e4.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- 3. INSTALL CONFIG & LAUNCH FOLDERS ---
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
