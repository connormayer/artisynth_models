# ============================================================
# Script: X-distance change between two markers per task,
#         and comparison between two conditions
# ============================================================

library(dplyr)
library(tidyr)
library(purrr)

# ------------------------------------------------------------
# 1. Markers of interest
# ------------------------------------------------------------
markers <- c(
  "marker4_xp116_yp16_zp110",  # back of the tongue
  "marker24_xp42_yp0_zp100"    # reference marker
)

# ------------------------------------------------------------
# 2. Function: compute X-distance change per task for ONE file
# ------------------------------------------------------------
compute_dist_by_task <- function(file, condition_label) {
  
  # Read the ArtiSynth position file
  data_raw <- read.csv(file, stringsAsFactors = FALSE)
  
  # Remove the dashed separator line and make task numeric
  data <- data_raw %>%
    filter(task != "------------------------------") %>%
    mutate(task = as.integer(task))
  
  # Keep only the two markers and their X coordinates
  marker_data <- data %>%
    filter(probe %in% markers) %>%
    select(task, probe, x)
  
  # Wide format: one row per task, X for each marker in its own column
  wide_x <- marker_data %>%
    pivot_wider(names_from = probe, values_from = x)
  
  # Initial configuration: task 0
  init_row <- wide_x %>%
    filter(task == 0) %>%
    slice(1)
  
  init_marker4_x  <- init_row$`marker4_xp116_yp16_zp110`
  init_marker24_x <- init_row$`marker24_xp42_yp0_zp100`
  init_dist_x     <- init_marker4_x - init_marker24_x
  
  # For each task, compute:
  # - initial vs final X for each marker
  # - initial vs final distance between markers along X
  summary_by_task <- wide_x %>%
    mutate(
      # initial X (same for all tasks)
      marker4_x_initial  = init_marker4_x,
      marker24_x_initial = init_marker24_x,
      
      # final X for this task
      marker4_x_final  = `marker4_xp116_yp16_zp110`,
      marker24_x_final = `marker24_xp42_yp0_zp100`,
      
      # change in X from initial (task 0) to this task
      marker4_dx  = marker4_x_final  - marker4_x_initial,
      marker24_dx = marker24_x_final - marker24_x_initial,
      
      # distance between markers along X
      initial_dist_x = init_dist_x,
      final_dist_x   = marker4_x_final - marker24_x_final,
      
      # how much the X-distance changed for this simulation
      change_dist_x  = final_dist_x - initial_dist_x,
      
      # label the condition (bones on/off, etc.)
      condition = condition_label
    ) %>%
    # drop task 0 so you only have the simulations
    filter(task != 0) %>%
    arrange(task)
  
  return(summary_by_task)
}

# ------------------------------------------------------------
# 3. Use it on ONE file (optional: just to inspect one condition)
# ------------------------------------------------------------

dist_one_condition <- compute_dist_by_task(
  file = "position.default.deactivated.txt",   # <--- change if needed
  condition_label = "bones_deactivated"
)

print(dist_one_condition)

write.csv(
  dist_one_condition,
  "dist_x_bones_deactivated.csv",
  row.names = FALSE
)

# ------------------------------------------------------------
# 4. Compare TWO conditions
#    Example: bones activated vs bones deactivated
# ------------------------------------------------------------

files <- c(
  "position.default.activated.txt",    # replace with your real file
  "position.default.deactivated.txt"   # replace with your real file
)

conditions <- c(
  "bones_activated",
  "bones_deactivated"
)

dist_compare_long <- map2_dfr(
  files,
  conditions,
  compute_dist_by_task
)

# Long format = one row per (task, condition)
print(dist_compare_long)

# ------------------------------------------------------------
# 4a. NEW: Check which tasks exist in which condition
# ------------------------------------------------------------
task_condition_table <- dist_compare_long %>%
  count(task, condition) %>%
  tidyr::pivot_wider(
    names_from  = condition,
    values_from = n,
    values_fill = 0
  )

# If you want to see only tasks that are missing from one condition:
print(
  task_condition_table %>%
    filter(bones_activated == 0 | bones_deactivated == 0)
)

# ------------------------------------------------------------
# 5. Make a side-by-side comparison per task
# ------------------------------------------------------------
dist_compare_wide <- dist_compare_long %>%
  select(task, condition, change_dist_x) %>%
  pivot_wider(
    names_from  = condition,
    values_from = change_dist_x
  ) %>%
  arrange(task) %>%
  mutate(
    # positive value = activated has more X-distance change than deactivated
    diff_activated_minus_deactivated =
      bones_activated - bones_deactivated
  )

print(dist_compare_wide)

write.csv(
  dist_compare_wide,
  "dist_x_comparison_activated_vs_deactivated.csv",
  row.names = FALSE
)

# ------------------------------------------------------------
# 5a. NEW: Only compare tasks where BOTH conditions exist
# ------------------------------------------------------------
dist_compare_common <- dist_compare_wide %>%
  filter(
    !is.na(bones_activated),
    !is.na(bones_deactivated)
  )

# Now these two vectors are the same length
all.equal(
  dist_compare_common$bones_activated,
  dist_compare_common$bones_deactivated
)
