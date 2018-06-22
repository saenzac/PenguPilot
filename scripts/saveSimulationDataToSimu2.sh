python /Volumes/Data/GoogleDrive/Data/Maestria/Investigation/tesis/PenguPilot/scripts/saveSimulationDataTo.py > simu2.log
cp simu2.log /Volumes/Data/GoogleDrive/Data/Maestria/Investigation/tesis/projekt_Qcopter/documentation/control/data_measurements/implementation/
cd /Volumes/Data/GoogleDrive/Data/Maestria/Investigation/tesis/projekt_Qcopter/documentation/control/data_measurements/implementation/
../py_parser/parse_fix.sh simu2.log

