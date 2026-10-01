import sys

def main(argv):
    if len(argv) < 2:
        print("Did not provide an input file")
        exit(1)
    
    if argv[1][-4:] != ".csv":
        print("Did not provide csv file")
        exit(1)

    f_in = open(argv[1], 'r')
    f_name = argv[1].split('.')
    f_out = open(f_name[0] + "-Filtered.csv", 'w+')

    lines = f_in.readlines()

    f_out.write(lines[0])

    lines = lines[1:]

    to_output = []

    for i in range(len(lines)):
        lines[i] = lines[i].split(',')

    for i in range(len(lines)):
        valid = True

        if lines[1] == '1' and lines[2] != '1':
            valid = False
        
        if valid:
            to_output.append(lines[i])
        
    for row in to_output:
        if len(row) != 0:
            for i in range(len(row)):
                f_out.write(f"{row[i].strip()}")
                if i != len(row) - 1:
                    f_out.write(',')
            f_out.write('\n')
    f_out.close()
        



if __name__ == '__main__':
    main(sys.argv)