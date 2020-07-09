import multiprocessing
def func(n):
    for i in range(10000):
        for j in range(10000):
            s=j*i
    print(n)
if __name__ == '__main__':
    pool = multiprocessing.Pool(processes=4)
    pool.map(func, range(10))
    pool.close()
    pool.join()
    print('done')


# Create class that will contain all variables I need in func
class PatientParamStruct:
    condition_timestamps = None # list cannot be initialized here!
    feature_numbers = None
    input_feature_folder = ''
    input_folder_validation = ''
    lag_number_of_windows = 0
    lag_time = 0
    lead_time = 0
    moving_window_time = 0
    output_folder = ''
    patient_number = 0
    prediction_window = 0
    samples_per_second = 0
    subwindow_lag_time = 0
