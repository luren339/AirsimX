# About the Star chart

Both READMEs embed the [Star History date chart for naipi11/AirsimX](https://www.star-history.com/#naipi11/AirsimX&Date). The image is served by `api.star-history.com`; it does not read the CSV in this repository or require a scheduled commit. For a repository with no stars there is no growth curve to plot. If the image is unavailable or cached, check the [current count on GitHub](https://github.com/naipi11/AirsimX/stargazers).

## Optional offline observations

The [CSV table](media/stars.csv) is a separate record of GitHub's public `stargazers_count` for this repository. Dates are UTC. On each observation day the most recent count replaces that day's previous value. Days without a reading are not reconstructed.

To make a new observation, run from this checkout with Python 3.12 and GitHub CLI configured for public repository reads:

```powershell
python -I tools/airsimx/community/update_stars.py --observe
```

Omit `--observe` to render the optional snapshot SVGs from the CSV entirely offline. This command does not update the Star History embed on the homepages.
