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
my_top_percentile = 0.025 # Percentage of outliers to remove from the top of the data, increase adjust until you data looks cleaned.
my_bottom_percentile = 0.025 # Percentage of outliers to remove from the bottom of the data
save_plots_to_file = True   # Set to True to save the plots to a file
plot_sampling_percentage = 1 # Adjust if need to plot faster, 1.0 = 100% of the data, 0.5 = 50% of the data, 0.1 = 10% of the data, etc.
figure_size = (10, 10) # Adjust the size of all the plots, (width, height)  
file_name = "mag_out_D_free4.csv" 
do_soft_iron = False

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
    
    print(f"Calibrating with {len(data)} data points...")
    # magnetic field strength, in microTesla
    F = 1000.0
    print(f"Magnetic field strength: {F} microTesla")
    b = np.zeros([3, 1])  # Hard iron bias
    A_1 = np.eye(3)       # soft iron bias
    
    # Ellipsoid fit
    s = data.T
    M, n, d = _ellipsoid_fit(s)
    
    # Calculate calibration parameters
    M_1 = linalg.inv(M)
    b = -np.dot(M_1, n)
    A_1 = np.real(F / np.sqrt(np.dot(n.T, np.dot(M_1, n)) - d) * linalg.sqrtm(M))
    
    print("\nCalibration completed!")
    print("Hard iron bias (microTesla):")
    print(f"  X: {b[0,0]:.6f}")
    print(f"  Y: {b[1,0]:.6f}")
    print(f"  Z: {b[2,0]:.6f}")
    print("\nSoft iron transformation matrix:")
    print(A_1)
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
    
    print(f"Calibrating with {len(data)} data points...")
    
    # Calculate hard iron bias as the mean of min and max values for each axis
    # This assumes the magnetometer was rotated through all orientations
    b = np.zeros([3, 1])
    
    for i, axis in enumerate(['X', 'Y', 'Z']):
        min_val = np.min(data[:, i])
        max_val = np.max(data[:, i])
        b[i, 0] = (max_val + min_val) / 2.0
        print(f"{axis}-axis: min={min_val:.2f}, max={max_val:.2f}, offset={b[i,0]:.2f}")
    
    print("\nCalibration completed!")
    print("Hard iron bias (offset in microTesla):")
    print(f"  X: {b[0,0]:.6f}")
    print(f"  Y: {b[1,0]:.6f}")
    print(f"  Z: {b[2,0]:.6f}")
    
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

    E_w, E_v = np.linalg.eig(E)
    v_1 = E_v[:, np.argmax(E_w)]
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
    return df

def filter_outlier(df, top_percentile=my_top_percentile, bottom_percentile=my_bottom_percentile):
    # Filter out the top 5% and bottom 5% values in each column to remove noise
    for column in ['x', 'y', 'z']:
        lower_bound = df[column].quantile(0 + bottom_percentile/2)
        upper_bound = df[column].quantile(1 - top_percentile/2)
        df = df[(df[column] >= lower_bound) & (df[column] <= upper_bound)]
    return df

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
    """Print calibration parameters as rust code."""
    print("\n" + "="*50)
    print("Rust Code for calibration parameters:")
    print("="*50)
    print(f"let hard_iron_bias_x = {b[0,0]:.6f};")
    print(f"let hard_iron_bias_y = {b[1,0]:.6f};")
    print(f"let hard_iron_bias_z = {b[2,0]:.6f};")
    print()
    print(f"let soft_iron_bias_xx = {A_1[0,0]:.6f};")
    print(f"let soft_iron_bias_xy = {A_1[0,1]:.6f};")
    print(f"let soft_iron_bias_xz = {A_1[0,2]:.6f};")
    print()
    print(f"let soft_iron_bias_yx = {A_1[1,0]:.6f};")
    print(f"let soft_iron_bias_yy = {A_1[1,1]:.6f};")
    print(f"let soft_iron_bias_yz = {A_1[1,2]:.6f};")
    print()
    print(f"let soft_iron_bias_zx = {A_1[2,0]:.6f};")
    print(f"let soft_iron_bias_zy = {A_1[2,1]:.6f};")
    print(f"let soft_iron_bias_zz = {A_1[2,2]:.6f};")
    print("="*50)
# Main function
def main():
    # Step 1: Read data from "data.csv"
    original_df = load_clean_csv(file_name) # helper function to load and clean the data, makes sure only lines with x,y,z values are present
    
    # Step 2: Filter out the outliers and print some information about the data
    filtered_df = filter_outlier(original_df, 0.025, 0.025)
    print(f"Total samples:  {len(original_df)}\t\toutliers filtered: {len(original_df) - len(filtered_df)}/{my_top_percentile + my_bottom_percentile}%")
    
    # Step 3: Apply correction and plot the data
    if do_soft_iron:
        b, A_1 = calibrate(filtered_df)
    else:
        b = calibrate_magnetometer_offset(filtered_df)
        A_1 = np.eye(3)

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