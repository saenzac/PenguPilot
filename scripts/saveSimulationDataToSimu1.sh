python /Volumes/Data/GoogleDrive/Data/Maestria/Investigation/tesis/PenguPilot/scripts/saveSimulationDataTo.py > simu1.log
cp simu1.log /Volumes/Data/GoogleDrive/Data/Maestria/Investigation/tesis/projekt_Qcopter/documentation/control/data_measurements/implementation/
cd /Volumes/Data/GoogleDrive/Data/Maestria/Investigation/tesis/projekt_Qcopter/documentation/control/data_measurements/implementation/
../py_parser/parse_fix.sh simu1.log

