# Motion Profile Simulator

I don't have a test bot, so here's a simulator that assumes that the robot has closed-loop velocity PID.

### Changelog

* **October 22, 2019:** fixed more `double` to `int` issues and made quicksort work with sorting 2-dimensional arrays. The following methods in `PathGenerator` are confirmed to work:
    * the constructor
    * `segV`
    * `closestPoint`
    * `curvature`
    * `getPath`
    * `Magnitude`
    * `maxVelocity`
    * `quickSort`
    * `swap`
    * `updatePos`
* **October 22, 2019:** I need help with physics, namely tankdrive physics. I have no clue how uniform circular motion works.

* **October 28, 2019:** Spend the entire day figuring out how to install FRC programs on a Windows machine. Also, GitHub is being weird with how it's storing code here so I'll upload code somehwere else.
* **October 28, 2019-November 13, 2019:** Figured out how to implement evenly spaced spline interpolation. Fixed a `stackOverflow` error without the use of Stack Overflow. I also figured out how to do `tankDrive`. Class `Spline` is now complete.

* **November 19, 2019:** Figured out how to implement tankdrive physics. Finished two out of the four cases.

* **November 25, 2019:** There are eight cases. Finished them all. Physics is hard.

* **December 5, 2019:** Realized I did my position updates for `tankDrive` wrong. This is beautiful. Physics is hard. AHHH HELP MY CODE IS WRONG HELP