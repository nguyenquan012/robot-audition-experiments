#!/usr/bin/env python
#
# read HARK localization output and calculate + display AOA in degree
#
import pylab
import sys
import math
import numpy
azimuths = []
azimuths1 = []
azimuths2 = [] 
azimuths3 = [] 
azimuths4 = [] 
source1 = []
source2 = [] 
source3 = []
source4 = []
f = open(sys.argv[1])
fileout = open("aoa.txt" , "w")
i = 0
azimuth_sum1 = azimuth_sum2 = azimuth_sum3 = azimuth_sum4 = 0
k = 30

##Function to get azimuth angle
def get_azimuth(line):
    x_str=(line.partition('x=')[-1].rpartition('y')[0]).replace(',','.')
    y_str=(line.partition('y=')[-1].rpartition('z')[0]).replace(',','.')
    x_num = float(x_str[1:(len(x_str)-2)])
    y_num = float(y_str[1:(len(y_str)-2)])
    #print >>fileout, x_str , y_str, x_num, y_num
    azimuth = math.atan2(y_num, x_num)*180/math.pi     	## calculate azimuth angle: = atan(y,x)
    return azimuth

# Loop over each line in the file that has source position.
for line in f.readlines():
 if "position x" in line:
    line.split()
    #print line
    i += 1 
    # angle data of 1st source
    if i == 1:  							
	 source1.append(i)
	 azimuth1=get_azimuth(line)
	 print >>fileout, line.strip(), "azimuth1 =", azimuth1, "\n"
	 azimuths1.append(azimuth1)
	 azimuths.append(azimuth1)
	 azimuth_sum1 += azimuth1

    # angle data of 2nd source
    elif i == 2:  
	 source2.append(i)
	 azimuth2=get_azimuth(line)		
	 print >>fileout, line.strip(), "azimuth2 =", azimuth2, "\n"
	 azimuths2.append(azimuth2)
	 azimuths.append(azimuth2)
	 azimuth_sum2 += azimuth2

    # angle data of 3rd source
    elif i == 3:  						
	 source3.append(i)
	 azimuth3=get_azimuth(line)		
	 print >>fileout, line.strip(), "azimuth3 =", azimuth3, "\n"
	 azimuths3.append(azimuth3)
	 azimuths.append(azimuth3)
	 azimuth_sum3 += azimuth3
    elif (i > 3):
	 azimuth = get_azimuth(line)
	 if (azimuth1 -k)< azimuth < (azimuth1+k):
		source1.append(i)
		print >>fileout, line.strip(), "azimuth1 =", azimuth, "\n"
         	azimuths1.append(azimuth)
         	azimuths.append(azimuth)
		azimuth_sum1 += azimuth
	 elif (azimuth2 -k)< azimuth < (azimuth2+k):
		source2.append(i)
		print >>fileout, line.strip(), "azimuth2 =", azimuth, "\n"
	 	azimuths2.append(azimuth)
	 	azimuths.append(azimuth)
		azimuth_sum2 += azimuth
	 elif (azimuth3-k) < azimuth < (azimuth3+k):
		source3.append(i)
		print >>fileout, line.strip(), "azimuth3 =", azimuth, "\n"
		azimuths3.append(azimuth)
		azimuths.append(azimuth)
		azimuth_sum3 += azimuth
	 else:
		source4.append(i)
		print >>fileout, line.strip(), "azimuth4 =", azimuth, "\n"
		azimuths4.append(azimuth)
		azimuths.append(azimuth)
		azimuth_sum4 += azimuth
fileout.close()
#print "i=",i
print "sample 1 =", len(source1)    # number of samples for each source
print "sample 2 =", len(source2)
print "sample 3 =", len(source3)
print "azimuth1=", azimuth1
print "azimuth2=", azimuth2
print "azimuth3=", azimuth3
print "azimuth sum 1=", azimuth_sum1
print "azimuth sum 2=", azimuth_sum2
print "azimuth sum 3=", azimuth_sum3
mean1 = azimuth_sum1/len(source1)
mean2 = azimuth_sum2/len(source2)
mean3 = azimuth_sum3/len(source3)

print "mean1:", azimuth_sum1/len(source1)
print "mean2:", azimuth_sum2/len(source2)
print "mean3:", azimuth_sum3/len(source3)
#print "mean4:", azimuth_sum4/len(source4)


##Plot aoa
pylab.figure(figsize=(8,6))
pylab.subplot(2, 1, 1)
pylab.plot(source1, azimuths1,'rp',label='aoa1 ~ %.2f' %mean1)
pylab.plot(source2, azimuths2,'cs',label='aoa2 ~ %.2f' %mean2)
pylab.plot(source3, azimuths3,'yo',label='aoa3 ~ %.2f' %mean3)
if len(source4)>0:
	mean4 = azimuth_sum4/len(source4)
	print "mean4:", azimuth_sum4/len(source4)
	pylab.plot(source4, azimuths4,'y*',label='aoa4 ~ %.2f' %mean4)
pylab.legend(loc=(1,0))
#pylab.plot(source1, azimuths1,'rp', source2, azimuths2,'cs', source3, azimuths3,'gx', source4, azimuths4,'y*')
pylab.axis([0, i,-180, 180])
pylab.yticks(numpy.arange(-180,180,20))
pylab.ylabel('Azimuth angle')
pylab.grid(True)
#pylab.text(10, 180, 'a1=%.2f'%mean1)


pylab.subplot(2, 1, 2)
#x,y,o = pylab.hist(pylab.array(azimuths))
x,y,o = pylab.hist(pylab.array(azimuths),bins=50,orientation='horizontal');
pylab.yticks(numpy.arange(-180,180,20))
pylab.xticks(numpy.arange(0,x.max(),5))

pylab.savefig('aoa.png', bbox_inches='tight')
pylab.show()

