Tomorrow: 
- [x] clean std:: functions in main file;
- [!] add functions for my load configuration ("my dirty changes" part) 
      and for motor assignments; # this step was considered to be irrelevant.
- [x] add bag-files to record the data;
- [ ] log pid:
      plotJuggle velocity with input command: 
       /diff_drive_plugin/cmd_vel
       /diff_drive_plugin/odom;
- [ ] adjust pid if needeed;
- [ ] add new high level pid for navigation:
      - [ ] create new pid class;
      - [ ] check with previously made class;
      - [ ] make two:
        - [ ] angle function;
        - [ ] line following;
