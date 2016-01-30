INTRO

  Contained in this directory are example karl scripts that have been
  written in a verbose fashion to facilitate debugging and modification.
  The *.mf files contain KaRL scripts for initialization of agents and
  periodic checks of a line intersection protocol between an attacker
  and a protected asset and interactions with protector agents. This may
  prove useful to anyone wanting to measure the success or failure of
  defensive algorithms using the karl interpreter (MADARA tool).


SPREADSHEET (agent_scenarios.xlsx)

  The spreadsheet contains visualizations of the initialization scripts.
  This should help with understanding what is being checked. Tabs are
  organized by agents_init_*.mf.


HOW TO RUN

  Each agents_init_success_*.mf is a specific scenario outlined in the
  agent_scenarios.xlsx spreadsheet. These files are provided to karl
  as initialization files (only evaluated once) in order to seed agent
  locations. The karl_zone_coverage_check.mf script is periodically
  evaluated by karl to see if the protectors are in between an attacker
  and base.

  The following examples showcase how to run the scripts. mission.failure
  is the key value to look for and a non-zero value is what the periodic
  check will be keying on (in order to short circuit if the mission fails)

  SUCCESSES

    karl -0f agents_init_success_1.mf -i karl_zone_coverage_check.mf -k
    karl -0f agents_init_success_2.mf -i karl_zone_coverage_check.mf -k

  FAILURES

    karl -0f agents_init_failure_1.mf -i karl_zone_coverage_check.mf -k
    karl -0f agents_init_failure_2.mf -i karl_zone_coverage_check.mf -k
    karl -0f agents_init_failure_3.mf -i karl_zone_coverage_check.mf -k


LIMITATIONS

  At the time of the writing of this document, the KaRL script was only
  setup to test for one attacker and one defender. This can be extended
  by making a for loop for the number of attackers and defenders within
  the karl_zone_coverage_check.mf, similar to the loop for the protectors.
