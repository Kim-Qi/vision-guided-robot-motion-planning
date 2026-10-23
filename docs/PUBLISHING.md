# Publication handoff

Base: `origin/main` at `9d2470fab3367f9489f10ca8ea455b953d4bebb5`; working branch: `feat/multi-object-pick-place`. The independent clone retains existing history. No force push, history rewrite, or Git init of the AutoDL runtime is used.

The current environment has no GitHub token, credential-helper entry, SSH client key or authenticated gh session. Remote publishing is not claimed. Local commit, incremental format-patch and Git bundle are prepared outside the repository; final handoff lists their SHA/paths.

On an authenticated client, first confirm the signed-in identity and permission for `Kim-Qi/vision-guided-robot-motion-planning`. For example, with an already installed/authenticated GitHub CLI:

```bash
gh api user --jq .login
gh repo view Kim-Qi/vision-guided-robot-motion-planning --json owner,viewerPermission,url
git remote get-url origin
git fetch origin
git push -u origin feat/multi-object-pick-place
gh pr create --repo Kim-Qi/vision-guided-robot-motion-planning \
  --base main --head feat/multi-object-pick-place \
  --title 'feat: add multi-object vision-guided pick-and-place pipeline' \
  --body-file docs/PULL_REQUEST.md
```

These are handoff commands, not commands executed successfully here. A push rejection must be inspected and resolved normally; never force push. Do not put tokens into source, URLs, shell history or patch files.

Only after the final publication/merge commit is confirmed, create the annotated stable tag `v1.0.0-pick-place` at that confirmed commit and publish the Release with `docs/RELEASE_NOTES.md`. No stable Git tag is created in this handoff. Upload the prepared red/blue/green MP4s, evidence archive and SHA256SUMS from the sibling `release_assets/v1.0.0-pick-place/` directory using the Release UI or authenticated CLI. Update README/DEMONSTRATION with real asset URLs and completed status only after upload succeeds.
