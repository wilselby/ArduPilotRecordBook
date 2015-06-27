
## ArduPilot LogBook

This program processes ArduPilot telemtery logs and creates a Flight Log Book or updates an existing flight logbook. The input files are in .csv file format. These are converted from the .tlog files developed by MissionPlanner. To convert the .tlog files to the required .csv files, load the .tlog in Misson Planner, select 'Tlog > Kml or Graph' then 'Convert to CSV'.

The program will output to the console status messages and error files. For each input file processed, it will display the following:

Date:  
MAV Type:  
Autopilot:   
Total Distance:   
Elapsed Time:   

The output file will be a .csv file with the following columns:

Date, MAV Type, Autopilot Type, LOS/FPV, # Take offs, Time (Hours:Minutes:Seconds), Distance (km)


## Installation

	make clean
	make

## Usage

	Logbook [options] [arg_name]

	-h, --help 			Displays usage syntax and exits
	-i [input_file(s)]	The input .csv file names/paths to be processed
	-n [output_file]	Name of the .csv Logbook file to be created
	-a [output_file]	Name of the exisisting .csv Logbook file to be updated

## Contributing

1. Fork it!
2. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Submit a pull request 

## History

v0.3 - Read and update existing Logbook files  
v0.3 - Added sitance and time totals computation  
v0.2 - Added command line inputs  
v0.1 - Process a single file  

## Credits

Author: Wil Selby

## License

