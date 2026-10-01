

def main():
    signals = {''}
    for i in range(1, 49):
        if i in {16,17,18,19,26,27,28,29,30}:
            continue
        else:
            filePath = f"Data/FO-LOG{i}.txt"
        
        file = open(filePath, 'r')
        for line in file:
            if line[0] != '\t':
                signals.add(line)
    sortedSignals = []
    for x in signals:
        sortedSignals.append(x)
    sortedSignals.sort()
    for x in sortedSignals:
        print(x)


    

if __name__ == '__main__':
    main()