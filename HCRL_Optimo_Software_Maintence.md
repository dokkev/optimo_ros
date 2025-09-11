## Optimo Hardware Config
Roboligent has assigned serial numbers to each Optimo Robot. Depending on the robot model, `optimo_api/resources/master0/OR7_config.yml` is unique to the specific hardware and should not be modified.

- AHG Optimo: `OR70003-1738945376`
- HCRL Optimo: `TBD`

Therefore resources directory is not tracked in git as it's defined inside `.gitignore` file. Instead, `optimo_api/OR7_config_backup` contains the backup config file for HCRL and AHG Optimo.    

In addition, AHG Optimo PC and HCRL Optimo PC have different roboligent sdk version (This needs to be fixed in future with roboligent's assistance). `optimo_api/CMakeLists.txt` is not tracked in git as well.