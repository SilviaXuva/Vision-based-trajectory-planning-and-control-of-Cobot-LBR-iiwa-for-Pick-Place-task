import time
import glob
import shutil
import os
import numpy as np
import pandas as pd
from Helpers.paths import Paths

def CleanUp():
    time.sleep(5)
    try:
        recordingFile = glob.glob(r'*.avi')[0]
        shutil.move(recordingFile, fr'{Paths.execution}\recording.avi')
    except Exception as e:
        print('Recording file not found')

    dataTypes = ['Traj', 'Real', 'Ref']
    measures = [f'{m}{measure}'for m in ['q','x'] for measure in ['','Dot','DotDot']]

    for obj in [os.path.basename(folder) for folder in glob.glob(fr'{Paths.execution}\Motion\*[!Data]')]:
        for dataType in dataTypes:
            for measure in measures:
                dfList = []
                for file in glob.glob(fr'{Paths.execution}\Motion\Data\{obj}_*_{measure}_{dataType}.csv'):
                    dfList.append(pd.read_csv(file, index_col=0))
                    if measure == doc:
                        os.makedirs(fr'{Paths.execution}\Motion\Doc\Data', exist_ok=True)
                        shutil.copyfile(file,fr'{Paths.execution}\Motion\Doc\Data\{os.path.basename(file)}')
                concat = pd.concat(dfList, ignore_index=True)
                concat.index = np.array([i*0.05 for i in range(len(concat.index))])
                concat.to_csv(fr'{Paths.execution}\Motion\Data\{obj}_0.Entire_{measure}_{dataType}.csv')
