What to do when things do not work
=========================================

- When Choreonoid is launched, the following error appears: A NameService doesn’t exist at “corbaloc:iiop:localhost:2809/NameService”.
- A CORBA error is displayed.
- A sample OpenRTM project does not work.
- An OpenRTM controller will not start.
 
The issues above can sometimes be solved by restarting the name server.

If you are using Linux, you can use the shell script **reset-omninames.sh** to restart the name server. The script is installed in **/bin** in the Choreonoid install location. Execute it as follows: ::

 reset-omninames.sh





