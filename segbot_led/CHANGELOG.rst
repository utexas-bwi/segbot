^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package segbot_led
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.5 (2016-08-27)
------------------
* Added need assistance animation and made circular 4-set animation default turn animation
* Added additonal circular variations to turn signals
* Updated led segments and blocked animation.  (`#73
  <https://github.com/utexas-bwi/segbot/issues/73>`_)
* Added install target for launch directory.
* Contributors: FernandezR, Rolando Fernandez

0.3.4 (2016-08-08)
------------------
* Added variations for turn signals and adjusted led segments
* Changed blocked animation to use a pulsing red led animation
* Renamed segbot_led launch for the version 3 build
* Moved messages over to bwi_msgs, made modifications to error
  handling logic, and condensed service advertisers.
* Fixed compile time message generation error
* Migrating LED control package from bwi_common to segbot.
* New segbot_led package
* Contributors: FernandezR, Jack O'Quin
