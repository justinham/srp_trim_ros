import asyncio
import can
import cantools
import sys



class CanReader:
    read = True
    async def waitForInput():
        user_input = ""
        user_input = await asyncio.to_thread(input, "Press enter to quit program\n")
        
        CanReader.read = False
        return False


    async def readCan():

        # argv[1] = Virtual CAN Network Name
        # argv[2] = DBC Name
        # argv[3] = filePath


        can.rc['interface'] = 'socketcan'
        can.rc['channel'] = sys.argv[1]
        can.rc['bitrate'] = 5000000
        

        fNum = 0
        logger = can.Logger("Data/"+sys.argv[3]+ "/" + sys.argv[2] + "-" + str(fNum) + ".asc")
        c = 0
        while(CanReader.read):
            with can.Bus() as bus:
                for msg in bus:
                    c += 1
                    logger.log_event(msg)
                    if c > 50000:
                        c = 0
                        fNum += 1
                        f.close()
                        logger.stop()
                        logger = can.Logger("Data/"+sys.argv[3]+ "/" + sys.argv[2] + "-" + str(fNum) + ".asc")
                        print("FileSplit, Created log number {fNum}\n")


    async def main():

        await asyncio.gather(
            CanReader.waitForInput(),
            CanReader.readCan(),
        )

asyncio.run(CanReader.main())