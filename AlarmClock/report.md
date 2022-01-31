# Quest Name
Authors: Luke Atlas, Remo Smadja, Scott Hom

2019-09-19

## Summary

Question: How can you synchronize multiple ESP clocks with each other.

- Answer 1: The clocks can be synchronized by making one of the  ESPs
the master and one of them the slave. The master and the slave can
communicate over UART, Wifi, bluetooth, etc... The slave will be pogrammed
to request to time from the master.

- Answer 2: As well the clock can synchronized by time zone through
pulling the time from the same time zone via wifi. Pulling the time
from wifi would need special.h files.

## Evaluation Criteria



## Solution Design



## Sketches and Photos
<center><img src="./images/example.png" width="70%" /></center>
<center> </center>


## Supporting Artifacts
- [Link to repo]()
- [Link to video demo]()


## References

-----

## Reminders

- Video recording in landscape not to exceed 90s
- Each team member appears in video
- Make sure video permission is set accessible to the instructors
- Repo is private
