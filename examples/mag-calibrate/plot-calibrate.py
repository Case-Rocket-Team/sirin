# to run: uv run plot-calibrate.py

# Make sure to install pandas, numpy and matplotlib before running this script,  it should run automatically under jupyter notebook
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from io import StringIO
from scipy import linalg

# Be sure to remove your outliers and check the data to see if it looks correct. ideally there should not any dots far from the "circles".
# Fraction of samples to drop as outliers: the ones whose distance from the first-pass center is furthest from
# the typical radius. Increase until the plots look clean.
outlier_fraction = 0.05
save_plots_to_file = True   # Set to True to save the plots to a file
plot_sampling_percentage = 1 # Adjust if need to plot faster, 1.0 = 100% of the data, 0.5 = 50% of the data, 0.1 = 10% of the data, etc.
figure_size = (10, 10) # Adjust the size of all the plots, (width, height)
file_name = "mag_out_D_free4.csv"
do_soft_iron = True  # False = hard iron only (per-axis min/max midpoint), soft iron = identity

LSB_PER_GAUSS = 6842.0  # LIS3MDL at +-4 gauss; the CSV is already in uT (raw / 6842 * 100)
UT_PER_GAUSS = 100.0

def apply_calibration(df, b, A_1):
    """
    Apply calibration to magnetometer data using hard iron bias and soft iron transformation.
    
    Args:
        df: pandas DataFrame with 'x', 'y', 'z' columns
        b: hard iron bias vector (3x1)
        A_1: soft iron transformation matrix (3x3)
    
    Returns:
        corrected_df: DataFrame with calibrated data
    """
    # Extract data as numpy array
    data = df[['x', 'y', 'z']].values
    
    # Apply calibration: (data - b^T) @ A_1^T
    data_corrected = (data - b.T) @ A_1.T
    
    # Convert back to DataFrame
    corrected_df = pd.DataFrame({
        'corrected_x': data_corrected[:, 0],
        'corrected_y': data_corrected[:, 1],
        'corrected_z': data_corrected[:, 2]
    })
    return corrected_df
        

def calibrate(df):
    """
    Perform magnetometer calibration using ellipsoid fitting.
    
    Args:
        df: pandas DataFrame with 'x', 'y', 'z' columns containing magnetometer data
    """
    # Convert dataframe to numpy array
    data = df[['x', 'y', 'z']].values

    # Ellipsoid fit: all points satisfy  m^T M m + 2 n^T m + d = 0
    s = data.T
    M, n, d = _ellipsoid_fit(s)

    # Hard iron = ellipsoid center
    M_1 = linalg.inv(M)
    b = -np.dot(M_1, n)

    # Rewritten around the center:  (m - b)^T M (m - b) = K,  K = n^T M^-1 n - d
    K = (np.dot(n.T, np.dot(M_1, n)) - d).item()
    # A_unit maps the ellipsoid onto the unit sphere: |A_unit (m - b)| = 1
    A_unit = np.real(linalg.sqrtm(M)) / np.sqrt(K)

    # Rescale so det(A_1) = 1: A_1 only reshapes the ellipsoid into a sphere and keeps its volume,
    # so the output stays in the sensor's own units (uT here, gauss on the rocket).
    A_1 = A_unit / np.cbrt(np.linalg.det(A_unit))
    return b, A_1

def calibrate_magnetometer_offset(df):
    """
    Perform magnetometer calibration using simple offset correction (hard iron only).
    
    Args:
        df: pandas DataFrame with 'x', 'y', 'z' columns containing magnetometer data
    
    Returns:
        b: Hard iron bias vector (3x1 numpy array)
    """
    # Convert dataframe to numpy array
    data = df[['x', 'y', 'z']].values

    # Calculate hard iron bias as the mean of min and max values for each axis
    # This assumes the magnetometer was rotated through all orientations
    b = ((data.max(axis=0) + data.min(axis=0)) / 2.0).reshape(3, 1)
    return b

def _ellipsoid_fit(s):
    ''' Estimate ellipsoid parameters from a set of points.

        Parameters
        ----------
        s : array_like
            The samples (M,N) where M=3 (x,y,z) and N=number of samples.

        Returns
        -------
        M, n, d : array_like, array_like, float
            The ellipsoid parameters M, n, d.

        References
        ----------
        .. [1] Qingde Li; Griffiths, J.G., "Least squares ellipsoid specific
            fitting," in Geometric Modeling and Processing, 2004.
            Proceedings, vol., no., pp.335-340, 2004
    '''
    # D (samples)
    D = np.array([s[0]**2., s[1]**2., s[2]**2.,
                    2.*s[1]*s[2], 2.*s[0]*s[2], 2.*s[0]*s[1],
                    2.*s[0], 2.*s[1], 2.*s[2], np.ones_like(s[0])])

    # S, S_11, S_12, S_21, S_22 (eq. 11)
    S = np.dot(D, D.T)
    S_11 = S[:6,:6]
    S_12 = S[:6,6:]
    S_21 = S[6:,:6]
    S_22 = S[6:,6:]

    # C (Eq. 8, k=4)
    C = np.array([[-1,  1,  1,  0,  0,  0],
                    [ 1, -1,  1,  0,  0,  0],
                    [ 1,  1, -1,  0,  0,  0],
                    [ 0,  0,  0, -4,  0,  0],
                    [ 0,  0,  0,  0, -4,  0],
                    [ 0,  0,  0,  0,  0, -4]])

    # v_1 (eq. 15, solution)
    E = np.dot(linalg.inv(C),
                S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))

    # E is not symmetric, so eig may hand back complex dtype; the eigenvector we want is real
    E_w, E_v = np.linalg.eig(E)
    v_1 = np.real(E_v[:, np.argmax(np.real(E_w))])
    if v_1[0] < 0: v_1 = -v_1

    # v_2 (eq. 13, solution)
    v_2 = np.dot(np.dot(-np.linalg.inv(S_22), S_21), v_1)

    # quadratic-form parameters, parameters h and f swapped as per correction by Roger R on Teslabs page
    M = np.array([[v_1[0], v_1[5], v_1[4]],
                    [v_1[5], v_1[1], v_1[3]],
                    [v_1[4], v_1[3], v_1[2]]])
    n = np.array([[v_2[0]],
                    [v_2[1]],
                    [v_2[2]]])
    d = v_2[3]

    return M, n, d

def load_clean_csv(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
    start_line = 0
    end_line = len(lines)

    # Filter lines and verify each line
    filtered_lines = []
    for line in lines[start_line:end_line]:
        if len(line.split(',')) == 3:  # Verify if the line contains exactly three parameters (x, y, z)
            filtered_lines.append(line)
    
    # Create a DataFrame from the filtered lines
    df = pd.read_csv(StringIO(''.join(filtered_lines)))

    # The firmware logs faster than the 10 Hz sensor updates, so the same sample often appears
    # twice in a row. Drop the repeats so slow parts of the tumble don't get extra weight.
    df = df[(df != df.shift()).any(axis=1)].reset_index(drop=True)
    return df

def corrected_radius(df, b, A_1):
    """Distance of each corrected sample from the origin."""
    data = df[['x', 'y', 'z']].values
    return np.linalg.norm((data - b.T) @ A_1.T, axis=1)

def filter_outlier(df, b, A_1, fraction=outlier_fraction):
    # Drop the samples whose corrected radius is furthest from the typical radius.
    # (Trimming each axis separately would remove the extreme points that pin the ellipsoid down.)
    r = corrected_radius(df, b, A_1)
    deviation = np.abs(r - np.median(r))
    return df[deviation <= np.quantile(deviation, 1 - fraction)]

def report_quality(df, b, A_1):
    """Print how well the corrected samples sit on a sphere and how well they cover it."""
    data = df[['x', 'y', 'z']].values
    c = (data - b.T) @ A_1.T
    r = np.linalg.norm(c, axis=1)
    R = r.mean()
    spread = r.std() / R * 100
    print(f"\nCorrected radius: {R:.1f} uT (Earth's field is ~25-65 uT; check NOAA for your location)")
    print(f"Radius spread:    {spread:.1f} %  {'OK' if spread < 5 else 'HIGH - redo the tumble away from metal, or raise outlier_fraction'}")
    for i, axis in enumerate('XYZ'):
        lo, hi = c[:, i].min() / R, c[:, i].max() / R
        ok = lo < -0.8 and hi > 0.8
        print(f"{axis} coverage:       {lo:+.2f} .. {hi:+.2f} of radius  {'OK' if ok else 'POOR - point this axis further up and down'}")
    octants = np.bincount(((c > 0) * [1, 2, 4]).sum(axis=1), minlength=8)
    print(f"Samples per octant: {octants.tolist()}  {'OK' if octants.min() >= 0.02 * len(c) else 'POOR - some directions barely visited'}")

# Helper functions for plotting
def clean_filename(filename):
    # return filename.replace(' ', '_').replace('/', '_').replace('\\', '_').replace(':', '_')
    return filename.replace('/', '_').replace('\\', '_').replace(':', '_')

def calculate_distance_from_origin(x, y):
    return np.sqrt(x ** 2 + y ** 2)

def normalize(distances, min_val, max_val):
    min_dist, max_dist = np.min(distances), np.max(distances)
    return min_val + (max_val - min_val) * (distances - min_dist) / (max_dist - min_dist)

def plot_data(df, title, xlabel, ylabel,save_img=False):
    plt.figure(title,figsize=figure_size)
    plt.scatter(df.iloc[:, 0], df.iloc[:, 1], s=2, label='XY', color='red')
    plt.scatter(df.iloc[:, 0], df.iloc[:, 2], s=2, label='XZ', color='green')
    plt.scatter(df.iloc[:, 1], df.iloc[:, 2], s=2, label='YZ', color='blue')
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.legend()
    # Make the aspect ratio equal
    plt.axis('equal')
    if save_img:
        plt.savefig(f"{clean_filename(title)}.png")
        
    plt.show(block=False)

def print_rust_code(b, A_1):
    """Print calibration parameters as constants for examples/main/src/config.rs."""
    b_gauss = b[:, 0] / UT_PER_GAUSS
    row = lambda r: "[" + ", ".join(f"{v:.6f}" for v in r) + "]"
    print("\n" + "="*70)
    print("Paste into examples/main/src/config.rs:")
    print("="*70)
    print(f"// Magnetometer calibration from {file_name}. Apply as: m_cal = SI * (m_counts / {LSB_PER_GAUSS:.0f} - HI)")
    print(f"pub const MAG_HARD_IRON_BIAS_GAUSS: [f32; 3] = {row(b_gauss)};")
    print("pub const MAG_SOFT_IRON: [[f32; 3]; 3] = [ // unitless, row-major, det = 1")
    for r in A_1:
        print(f"    {row(r)},")
    print("];")
    print("="*70)
# Main function
def main():
    # Step 1: Read data from "data.csv"
    original_df = load_clean_csv(file_name) # helper function to load and clean the data, makes sure only lines with x,y,z values are present

    # Step 2: Rough ellipsoid fit on everything, only used to find outliers, then drop them
    b0, A0 = calibrate(original_df)
    filtered_df = filter_outlier(original_df, b0, A0)
    print(f"Samples: {len(original_df)}  outliers dropped: {len(original_df) - len(filtered_df)} ({outlier_fraction:.0%})")

    # Step 3: Real fit on the cleaned data
    if do_soft_iron:
        b, A_1 = calibrate(filtered_df)
    else:
        b = calibrate_magnetometer_offset(filtered_df)
        A_1 = np.eye(3)

    print(f"\nHard iron bias (uT): {np.round(b[:, 0], 3).tolist()}")
    print(f"Soft iron matrix (unitless):\n{np.round(A_1, 4)}")
    report_quality(filtered_df, b, A_1)

    filtered_corrected_df = apply_calibration(filtered_df, b, A_1)
    print_rust_code(b, A_1)
    # Step 5: Plot some information about the original data
    # Tip: Sampling the data with outliers is not a good idea, as the outliers may are likely to be removed and not plotted, for the other plots, sampling is fine but not required
    plot_data(original_df, title="Figure 1 - Data with outliers", xlabel="XY", ylabel="YZ", save_img=save_plots_to_file)
    # Step 7: Plot some information about the outlier cleaned data
    plot_data(filtered_df.sample(frac=plot_sampling_percentage), title="Figure 2 - Data without outliers", xlabel="XY", ylabel="YZ", save_img=save_plots_to_file)

    # Step 8: Plot some information about the corrected data
    plot_data(filtered_corrected_df.sample(frac=plot_sampling_percentage), title="Figure 3 - Clean data and offset", xlabel="Offset corrected XY", ylabel="Offset corrected YZ", save_img=save_plots_to_file)

if __name__ == "__main__":
    main()

plt.show()