# About the Star chart

The [CSV table](media/stars.csv) contains an observation of GitHub's public `stargazers_count` for **naipi11/AirsimX**, not for Microsoft AirSim or the maintainer's older fork. Dates are UTC. On each observation day the most recent count replaces that day's previous value. Days without a reading are not reconstructed. A count can go down as well as up.

To make a new observation, run from this checkout with Python 3.12 and GitHub CLI configured for public repository reads:

```powershell
python -I tools/airsimx/community/update_stars.py --observe
```

Omit `--observe` to render the two SVGs entirely offline from existing rows. Inspect the changes before committing them. The README never calls a third-party image service and does not imply the snapshot is live. There is no scheduled writer, no embedded token and no list of stargazer identities. The chart links to the [live GitHub count](https://github.com/naipi11/AirsimX/stargazers).
