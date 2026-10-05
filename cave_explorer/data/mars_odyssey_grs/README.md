# Mars Odyssey GRS water-concentration map

Real orbital measurements used as the ground-truth field for Advanced 6 (adaptive scientific sampling).

## Source

| | |
|---|---|
| Mission / instrument | NASA 2001 Mars Odyssey — Gamma Ray Spectrometer (GRS) |
| Data set | `ODY-M-GRS-5-ELEMENTS-V1.0`, "ODY Mars Gamma Ray Spectrometer 5 Element Concentration V1.0" |
| Product | `H2O_SR_5X5`, product version 2.0 |
| Archive | NASA Planetary Data System (PDS), Geosciences Node |
| URL | <https://pds-geosciences.wustl.edu/ody/ody-m-grs-5-elements-v1/odgm_xxxx/data/smoothed/> |
| Observation period | 4 June 2002 – 3 April 2005 |
| Downloaded | 5 October 2026 |
| SHA-256 of `h2o_sr_5x5.tab` | `5cec0d637755ddb7a451f58d7b24b6fb34f081e3391b40cdfca5410200da0236` |

## Files

- `h2o_sr_5x5.tab` — the data table, unmodified.
- `h2o_sr_5x5.lbl` — the PDS3 label describing the table, unmodified.

## Contents

Water (H2O) concentration in **weight percent** on a 5° × 5° latitude/longitude grid, smoothed by the instrument team with a 10° boxcar filter. 2592 rows (36 latitudes × 72 longitudes), five columns:

1. Latitude of bin centre [deg]
2. Longitude of bin centre [deg], 0–360
3. H2O concentration [wt %]
4. 1-sigma error [wt %]
5. 1-sigma error including correction-factor errors [wt %]

`9999.999` marks bins with no data. Valid data covers latitudes 57.5° S to 57.5° N (1508 bins), values 1.6 to 7.4 wt %.

## How the project uses it

`cave_explorer/advanced/measurement_field.py` takes a 12 × 12 bin patch (latitude 37.5° S – 17.5° N, longitude 212.5° – 267.5°) and stretches it over the cave's x/y extent. Within that patch the concentration ranges from 2.3 to 5.0 wt %. Measurement noise is drawn from the instrument's own reported 1-sigma error (column 4).

The patch is centred near Arsia Mons in the Tharsis region, where lava-tube skylights have been observed from orbit.

## Limits — state these in the report

- **The values and the noise levels are real; the spatial scale is not.** Each bin is about 300 km across on Mars; the patch covers roughly 3300 km and is compressed onto a cave tens of metres wide.
- **This is orbital surface data, not cave data.** GRS senses hydrogen in the top few tens of centimetres of soil from orbit. Nobody has measured water concentration inside a Martian cave.
- **Longitude convention.** The label does not state east or west longitude. East longitude was inferred by checking that the known water-rich equatorial regions (Arabia Terra, and the region near 180° E) appear where expected in the table.
- The task specification permits "data adapted from an existing source" that "does not need to realistically match the cave environment".
