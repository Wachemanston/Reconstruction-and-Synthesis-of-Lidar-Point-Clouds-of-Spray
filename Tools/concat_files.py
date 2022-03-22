import os
import shutil
from os import path
from os import listdir
from os.path import isfile, join

def main():
    input_directory = "your OutputFilterPath" # same value as setting .env
    output_file = "your output path" # ex: ./Desktop/filter_new.txt"

    if path.exists(output_file) :
        print("Filter file already exist")
        return

    pcd_files = [f for f in listdir(input_directory) if isfile(join(input_directory, f))]
    pcd_files.sort()

    with open(output_file, 'w+') as outfile:
        for pcd_file in pcd_files:
            with open(input_directory + pcd_file) as infile:
                for line in infile:
                    outfile.write(line)

		
if __name__ == "__main__":
    main()
