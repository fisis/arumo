#!/bin/bash
printf "NOTE: Run this script as 'sudo ./[script_file].sh'\n\n"

aruco_calibration_dict="14"
aruco_detection_dict="14"  # "6"

# ==============================================

board_params="-w=5 -h=6 -l=100 -s=25 -d=$aruco_calibration_dict"
board_file="./config.d/calibration_board.png"

calibration_reprojection_error="2.0"  # in pixels. ideally should e 1.0 or less
calibration_detector_paramfile="./config.d/marker_detector_params_default.yml"
calibration_params="-fmark=0.1 -frdiff=200 -nfcycle=25 -nfcalib=50 -rethresh=$calibration_reprojection_error"
calibration_params_file="./config.d/intrinsic_params_video[ci].yml"

# ----------------------------------------------

marker_dpi=200
page_width=1700  # 8.5 * mrker_dpi
page_height=2200  # 11 * marker_dpi
marker_edgelen_pix=1000
marker_edgelen_m=0.127  # (marker_side_pix / marker_dpi) * 0.0254

# ----------------------------------------------

create_marker_params="-d=$aruco_detection_dict -ms=$marker_edgelen_pix"
marker_file_prefix="./config.d/marker_"

transformation_detector_paramfile="./config.d/marker_detector_params_default.yml"
transformation_params="-d=$aruco_detection_dict -l=$marker_edgelen_m -fframe=0.75 -rmaxerr=1e-5 -tmaxerr=1e-8"
transformation_params_file="./config.d/transfomation_params_video[ci].yml"
transformation_ground_coord_default="0:(0,0,0);4:(4,0,0);25:(0,5,0);29:(4,5,0);i"  # corners of 5x6 calibration board

view_detector_paramfile="./config.d/marker_detector_params_default.yml"
view_params="-d=$aruco_detection_dict"

track_params="-d=$aruco_detection_dict -l=$marker_edgelen_m -mposeage=1.0"

# ==============================================

printf "Choose an action from below:
    
    (1) Create calibration board.
    (2) Create markers.
    ================================
    (3) View markers.
    --------------------------------
    (4) Calibrate camera.
    (5) Compute transformation.
    (6) Track markers.
\n"
read -p $'Press a number: ' -n 1 action
printf "\n"

# ==============================================

# Create calibration board.
if [ $action -eq "1" ]; then
    set -x
    ./bin/createboard $board_params $board_file

# Create markers.
elif [ $action -eq "2" ]; then
    read -p $'Enter marker ID: ' markid
    printf "\n"
    (set -x ;
    ./bin/createmarker $create_marker_params -id=$markid $marker_file_prefix$markid.png ;
    convert \( -gravity center $marker_file_prefix$markid.png -background white -extent ${page_width}x${page_height} -density ${marker_dpi}x${marker_dpi} -units pixelsperinch \) \
            \( -gravity northwest -pointsize 72 label:"$aruco_detection_dict:$markid" -geometry +50+50  \) \
            -composite $marker_file_prefix$markid.pdf )
    rm $marker_file_prefix$markid.png
    printf "Marker edge length in meters: $marker_edgelen_m \n"

# View markers.
elif [ $action -eq "3" ]; then
    read -p $'Enter camera ID: ' -n 1 camid
    printf "\n"
    set -x
    ./bin/viewmarkers $view_params -ci=$camid -dp=$view_detector_paramfile -c=$calibration_params_file

# Calibrate camera.
elif [ $action -eq "4" ]; then
    read -p $'Enter camera ID: ' -n 1 camid
    printf "\n"
    set -x
    ./bin/calibratecamera $board_params $calibration_params -ci=$camid -dp=$calibration_detector_paramfile $calibration_params_file

# Compute transformation.
elif [ $action -eq "5" ]; then
    read -p $'Enter camera ID: ' -n 1 camid
    printf "\n"
    read -p \
      $"Marker ground coordinates [
        Format: 'id1:(x1,y1,z1);id2:(x2,y2,z2);...;[i|u]'.
        Default: $transformation_ground_coord_default  (corners of 5x6 calibration board) 
      ]: " transformation_ground_coord
    if [ "$transformation_ground_coord" == "" ]; then
        transformation_ground_coord=$transformation_ground_coord_default
    fi
    set -x
    ./bin/computetransformation $transformation_params -ci=$camid -dp=$transformation_detector_paramfile -c=$calibration_params_file -grcoords=$transformation_ground_coord  $transformation_params_file

elif [ $action -eq "6" ]; then
    read -p $'Enter camera IDs (comma-separated list with no space): ' camids
    printf "\n"
    set -x
    ./bin/trackmarkers $track_params -ci=$camids -dp=$view_detector_paramfile -c=$calibration_params_file -t=$transformation_params_file

fi

