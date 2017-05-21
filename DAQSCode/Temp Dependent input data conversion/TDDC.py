sq = open('tddc.csv', mode='r')

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
print (data[0])

#WY Method for manipulating the data
class gas:
    def __init__(self, name, temp, coeff):
        self.GAS = GAS
        self.temp = temp
        self.coeff = coeff


    def calccoeff(self, temp, coeff):
       
            
    def __str__(self):
        return str(self.GAS), str(self.temp), str(self.coeff)

def main(datainput):



main(data)
sq.close()
