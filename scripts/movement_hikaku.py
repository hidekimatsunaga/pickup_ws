import sys
import pandas as pd
import matplotlib.pyplot as plt


def load_csv(path):
    df = pd.read_csv(path)
    return df


def main(csv1, csv2):
    df1 = load_csv(csv1)
    df2 = load_csv(csv2)

    plt.figure()
    plt.plot(df1['x'], df1['y'], label='run1')
    plt.plot(df2['x'], df2['y'], label='run2')
    plt.axis('equal')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.legend()
    plt.title('cmd_vel integrated trajectory (x-y)')

    # goal/center が入ってたら最後の値を点で載せる（任意）
    for df, name in [(df1, 'run1'), (df2, 'run2')]:
        if 'goal_x' in df.columns and df['goal_x'].notna().any():
            gx = df['goal_x'].dropna().iloc[-1]
            gy = df['goal_y'].dropna().iloc[-1]
            plt.scatter([gx], [gy], marker='x', label=f'{name} goal')
        if 'center_x' in df.columns and df['center_x'].notna().any():
            cx = df['center_x'].dropna().iloc[-1]
            cy = df['center_y'].dropna().iloc[-1]
            plt.scatter([cx], [cy], marker='o', label=f'{name} center')

    plt.legend()
    plt.show()


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('Usage: python plot_compare.py run1.csv run2.csv')
        sys.exit(1)
    main(sys.argv[1], sys.argv[2])
