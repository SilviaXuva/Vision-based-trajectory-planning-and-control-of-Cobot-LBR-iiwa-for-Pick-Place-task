import glob
import numpy as np
import pandas as pd
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def plotOutputs(folder, number_joints = 7):
    options = {
        'Cart Comparison':{
            'measure': 'x',
            'height': 5,
            'width': 15,
            'figures': [
                {
                    'data': [['x', 'y', 'z']],
                    'name': 'Trans'
                }, 
                {
                    'data': [['rx', 'ry', 'rz']],
                    'name': 'Rot'
                }
            ]
        },
        'Joint Comparison': {
            'measure': 'q',
            'height': 30,
            'width': 15,
            'figures': [
                {
                    'data': [[f'q{i}'] for i in range(number_joints)],
                    'name': 'All joints'
                }
            ]
        }
    }

    comparison = [
        {
            'Real': 'red',
            'Ref': 'blue'
        }, 
        {
            'Real': 'red',
            'Traj': 'blue'
        }
    ]

    for targetFolder in glob.glob(fr'{folder}\Motion\*\*'):
        for key in list(options.keys()):
            for measure in ['', 'Dot', 'DotDot']:
                for compare in comparison:
                    for figure in options[key]['figures']:
                        rows, cols = np.array(figure['data']).shape
                        fig, ax = plt.subplots(rows, cols)
                        fig.suptitle(f'{figure["name"]}: {options[key]["measure"]}{measure}')
                        fig.set_figheight(options[key]['height'])
                        fig.set_figwidth(options[key]['width'])
                        try:
                            for i, df_column in enumerate(np.array(figure['data']).flatten()):
                                for item in compare.items():
                                    dataType, color = item
                                    ax = plt.subplot(rows, cols, i + 1)
                                    file = fr'{targetFolder}\{key}\Data\{options[key]["measure"]}{measure}\{options[key]["measure"]}{measure}_{dataType}.csv'
                                    df = pd.read_csv(file, index_col=0)
                                    df = df.rename(columns={df_column: f'{df_column}{measure}_{dataType}'})
                                    df[f'{df_column}{measure}_{dataType}'].plot(x=df.index, ax=ax, legend=True, color=color)
                                    ax.title.set_text(f'{df_column}{measure}')
                            savePath = fr'{targetFolder}\{key}\Plot\{options[key]["measure"]}{measure}\{'-'.join(compare.keys())}'
                            os.makedirs(savePath, exist_ok=True)
                            fig.savefig(fr'{savePath}\{options[key]["measure"]}{measure}_{figure["name"]}.png')
                            plt.close()
                        except:
                            pass
