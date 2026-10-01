import cantools
import sys

def main(input_file, output_file):
    f_in = open(input_file, 'r')
    f_out = open(output_file, 'w')
    count = 0

    prev_timestamp = 0
    
    total_t = 0

    # Reads the three starting lines, may need to automate this if the number varies by file
    line = f_in.readline()
    line = f_in.readline()
    line = f_in.readline()
    f_out.write(f"timestamp delta_timestamp can_network frame_id data\n")
    while True:
        
        # Get next line from file
        line = f_in.readline().lstrip(' ')
        #print(line)
        # if line is empty
            # end of file is reached
        if not line:
            break

        # Get the time stamp
        timestamp = line[:line.index(' ')]
        line = line[line.index(' ')+1:]
        

        # Ignore Heartbeat messages
        if line[:line.index(' ')] == 'CANFD':
            
            # CANFD 8 Rx --- SKIPPING
            line = line[line.index(' '):].lstrip()
            network = line[:line.index(' ')]
            line = line[line.index(' '):].lstrip()

            line = line[line.index(' '):].lstrip()



            frame_id = line[:line.index(' ')].strip()

            if frame_id == "588" or frame_id == "54b":
                continue
            #print(frame_id)
            line  = line[line.index(' '):].lstrip()

            # There are three numbers, idk what they mean but we don't need them
            line = line[line.index(' '):].lstrip()
            line = line[line.index(' '):].lstrip()
            line = line[line.index(' '):].lstrip()

            data_len = line[:line.index(' ')]
            #print(data_len)
            line = line[line.index(' '):].lstrip()
            data = ""
            for i in range(int(data_len)):
                data += line[0:2]
                line = line[3:]
            #print(data)
            delta_timestamp = float(timestamp) - prev_timestamp
            prev_timestamp = float(timestamp)
           
            total_t += round(delta_timestamp, 6)

            f_out.write(f"{timestamp} {round(delta_timestamp, 6):.6f} {network} {frame_id} {data}\n")
            count += 1
        elif line[:line.index(' ')] == '5':

            network = line[:line.index(' ')]
            line = line[line.index(' '):].lstrip()
            frame_id = line[:line.index(' ')]
            line = line[line.index(' '):].lstrip()

            # Skips Rx d
            line = line[line.index(' '):].lstrip()
            line = line[line.index(' '):].lstrip()
            data_len = line[:line.index(' ')]
            line = line[line.index(' '):].lstrip()

            data = ""
            for i in range(int(data_len)):
                data += line[0:2]
                line = line[3:]
            #print(data)
            delta_timestamp = float(timestamp) - prev_timestamp
            prev_timestamp = float(timestamp)
           
            total_t += round(delta_timestamp, 6)

            f_out.write(f"{timestamp} {round(delta_timestamp, 6):.6f} {network} {frame_id} {data}\n")
            count += 1
            
    print(f"Final Number of Elements: {count}")
    print(f"Total t: {total_t}")
    f_in.close()
    f_out.close()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Did not provide an input file!!!")
        exit(1)
    
    if sys.argv[1][-4:] != ".asc":
        print("Did not provide a .asc as exported from VehicleSpy")
    
    input_file = sys.argv[1]
    output_file = input_file[:-4] + "-Transformed.txt"
    main(input_file, output_file)