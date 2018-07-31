


# The filename where we write output to
outputfile = 'simpleschedule.txt'

T_OFFSET       = 3 # how many seconds before you start
N_TRIALS       = 50
TRIAL_DURATION = 5.5

# Generate the timing grid
trials = [ T_OFFSET + (t*TRIAL_DURATION) for t in range(N_TRIALS) ]



# Now let's write the schedule to the output file
outfile = open(outputfile,'w')
outfile.write('trial TR t.offset\n')
tr = 1
for trialn,trial in enumerate(trials):
    outfile.write('%i %i %f\n'%(trialn+1,tr,trial))
outfile.close()


print("Written output to %s"%outputfile)
