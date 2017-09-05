import os
import subprocess

ROOT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
ROS_SRC_DIR = os.path.realpath(os.path.join(ROOT_DIR, 'ros', 'src'))
PYJOBS_DIR = os.path.realpath(os.path.join(ROOT_DIR, 'pyjobs'))

N_MAX_ERRORS = 121


def main():
    output = 0
    cmd = ['docker', 'run', '--rm=true',
           '--volume={}:{}'.format(ROOT_DIR, ROOT_DIR),
           'eurobots/carnd_capstone',
           'pycodestyle', '--count', ROS_SRC_DIR, PYJOBS_DIR]

    try:
        subprocess.check_output(cmd, stderr=subprocess.STDOUT)

    except subprocess.CalledProcessError as e:
        output_str = e.output
        print(output_str)

        n_errors = int(output_str.splitlines()[-1])

        if n_errors > N_MAX_ERRORS:
            print('Exceeded maximum number of errors: {} > {}'
                  .format(n_errors, N_MAX_ERRORS))
            output = e.returncode

    return output


if __name__ == '__main__':
    exit(main())
