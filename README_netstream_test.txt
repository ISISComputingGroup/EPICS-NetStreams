This branch contains code hacked a bit to test NI network streams

If you run example.vi it will create two passive network streams on localhost, one for writing and one for reading

When the IOC is started it will create two of its own endpoints and then connect these to the labview ones
(see netvarconfig.xml for names)

You can start either the ioc or labview first, if you start the ioc it will will wait for labview to start
 

If you caput to process variable   TEST:icont1   it will appear in labview vi in "Stream Reader" indicator in the front panel

If you camonitor  TEST:ind1_RBV  you should see the contents of "Stream Writer" on the vi front panel


Typing the "dbior" command in the ioc window will give somne connection statistics


Currently there is no recovery on restart of either connection endpoint and only simple scalars (int, real, string) are supported.
Arrays will be added at a later date. 

