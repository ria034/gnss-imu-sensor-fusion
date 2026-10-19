# Camera Calibration & Image Mosaicing

MATLAB camera calibration and image-stitching/mosaicing exercises: Harris
corner detection, feature matching, and panorama stitching across several
image sets with different overlap percentages and scenes.

## Contents

- `src/harris.m` — Harris corner detector implementation.
- `src/convolve2.m` — 2D convolution helper used by `harris.m`.
- `src/image_mosaicing.m` — panorama stitching over `data/building_dir2/`.
- `src/image_mosaicing_overlap.m` — panorama stitching over `data/overlap_15/`
  (low-overlap case).
- `src/image_mosaicing_cinder_block.m` — panorama stitching over
  `data/building_dir_cinder_wall/`.
- `calibration/calib_example/` — camera calibration image set (`Image1.tif`
  … `Image17.tif`, plus `Image_01.jpeg`) and `calib_data.mat` (calibration
  result, e.g. intrinsics/extrinsics from the MATLAB Camera Calibrator app).
- `data/` — image sets used for mosaicing, by overlap/scene:
  - `overlap_15/`, `overlap_50/`, `overlap_50_v2/` — same scene captured at
    ~15% and ~50% frame overlap (two takes of the 50% set).
  - `building_dir/`, `building_dir2/` — two takes of a building facade.
  - `building_dir_cinder_wall/` — cinder-block wall scene.

## Running

Open in MATLAB and run one of the `image_mosaicing*.m` scripts directly —
each resolves its own image folder relative to its own location, so no path
edits are needed:

```matlab
cd camera_calibration_mosaicing/src
image_mosaicing_overlap        % stitches data/overlap_15/
image_mosaicing                % stitches data/building_dir2/
image_mosaicing_cinder_block   % stitches data/building_dir_cinder_wall/
```

Each script calls `harris.m` (which calls `convolve2.m`) to extract corner
features, matches them across image pairs with `extractFeatures`/
`matchFeatures`, estimates pairwise transforms, and composites the stitched
panorama with `imwarp`/`imblend`-style blending.

> The scripts originally hardcoded local Windows paths
> (`C:\Users\lenovo\Downloads\Camera Calibration Module\...`). These were
> repointed at the `data/` folders in this repo, resolved relative to the
> script's own location so they work regardless of your current directory.
