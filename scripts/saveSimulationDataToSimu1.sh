python /Volumes/data/GoogleDrive/Data/Maestria/Investigation/tesis/PenguPilot/scripts/saveSimulationDataTo.py > simu1.log
cp simu1.log /Volumes/data/GoogleDrive/Data/Maestria/Investigation/tesis/projekt_Qcopter/documentation/control/data_measurements/simu1/
cd /Volumes/data/GoogleDrive/Data/Maestria/Investigation/tesis/projekt_Qcopter/documentation/control/data_measurements/simu1
../py_parser/parse_fix.sh simu1.log

