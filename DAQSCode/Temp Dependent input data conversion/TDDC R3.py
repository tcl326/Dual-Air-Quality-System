sq = open('tddc.csv', mode='r')
import random

#WY Read in header of CSV file.
hdr = sq.readline()
hdr = hdr.strip().split(',')
#WY Create list (data) with each positions full data (temp and coefficient) as a single entry.
data = []
#WY Store each positions full data as a dictionary in each list position (keys = temp, values = coeff)
for line in sq:
    row = line.strip().split(',')
    d = {}
    for i in range(len(hdr)):
        key = hdr[i]
        value = row[i]
        d[key] = value
    data.append(d)

#Printing list of gases and corresponding codes.    
print('Gas Number Codes (A = low grade, B = high grade):')
gaslist = ['OX-A', 'OX-B', 'SO2-A', 'SO2-B', 'NO2-A', 'NO2-B', 'NO-A', 'NO-B', 'CO-A', 'CO-B', 'H2S-A', 'H2S-B']
for i in range(len(gaslist)):
      print (gaslist[i], '=', i)

def main(datainput):
    #g = gas()
    inGAS = int(input('Gas Number:'))
    #temp = str(round(int(input('Temp:')),-1))
    #Mock input temps
    for i in range(30):
        temp = str(round(20 + random.randint(-1, 1), -1))
        coeff = datainput[inGAS][temp]
        #YARA, please work on the following line. Look at the docs Enes put in the folder to
        #create an equation based on the input reading and the offset, given the coefficient
        #based on temp calculated above.
        output = #INPUT_HERE# coeff*#OFFSET#
        print(output)


main(data)
sq.close()
