# Defender

Defender for the Analogue Pocket.

* Based on the FPGA Defender code by [Dar]( http://darfpga.blogspot.fr)
* Ported from the [MiSTer port by Sorgelig](https://github.com/MiSTer-devel/Arcade-Defender_MiSTer)

## Compatibility

This core supports four games that are compatible with the "early" Williams 6809 arcade board. This includes:

* Defender (by Williams)
* Jin (by Falcon)
* Mayday (by Hoei)
* Colony 7 (by Taito)

Note that the arcade games supported on the later Rev 1 board (Joust/Robotron, etc.), or Rev 2 (Joust 2, etc.) 6809 board are *not* supported by this core.

## Special controls

The action buttons for each game can be remapped through the Analogue Pocket UI.

Some additional controls (for the high-score-reset and and service menu ) are mapped as follows:

* Advance -- Select + L
* Auto-up -- R
* Reset High Scores -- Select + R

## Usage

*No ROM files are included with this release.*  

Install the contents of the release to the root of the SD card.

Place the necessary `.rom` files for the supported games onto the SD card under `Assets/defender/common`.

To generate the `.rom` format binaries used by this core, you must use the MRA files included in this repo, along with the corresponding ROMs from the most recent MAME release.

## Known issues and limitations

* High-score saving is not supported.

## Notes

Note:  Some of these games make excessive use of strobe effects, which may be problematic for individuals with photosensitivity, epilepsy, or other similar conditions.

## History

v0.9.0
* Initial Release

## Attribution

```
---------------------------------------------------------------------------------
-- Arcade: Defender port to MiSTer by Sorgelig
-- 22 October 2017
-- 
---------------------------------------------------------------------------------
-- Defender by Dar (darfpga@aol.fr) (10 October 2017)
-- http://darfpga.blogspot.fr
--
---------------------------------------------------------------------------------
-- gen_ram.vhd
-------------------------------- 
-- Copyright 2005-2008 by Peter Wendrich (pwsoft@syntiac.com)
-- http://www.syntiac.com/fpga64.html
---------------------------------------------------------------------------------
-- cpu09l - Version : 0128
-- Synthesizable 6809 instruction compatible VHDL CPU core
-- Copyright (C) 2003 - 2010 John Kent
---------------------------------------------------------------------------------
-- cpu68 - Version 9th Jan 2004 0.8
-- 6800/01 compatible CPU core 
-- GNU public license - December 2002 : John E. Kent
----------------------------------------------------------------

```
