#name of example
ex=inverted_pendulum

#compile
make
#remove any old output
if [ -d out ]; then
    rm -r out
fi
#create an output directory
mkdir out
#run the integration
./${ex}.exe
#plot results
python3 plot_${ex}.py
#remove stuff
rm -r ./${ex}.exe out
