#!/usr/bin/env node
// Lightweight structural check for the single-file embedded UI.

import fs from "node:fs";

const file = process.argv[2];
if (!file) {
  console.error("usage: node check_web_page.mjs <web_page.html>");
  process.exit(2);
}

const html = fs.readFileSync(file, "utf8");
const scripts = [...html.matchAll(/<script[^>]*>([\s\S]*?)<\/script>/gi)].map((match) => match[1]);
if (!scripts.length) throw new Error("no inline script found");
for (const script of scripts) new Function(script);

const ids = [...html.matchAll(/\bid="([^"]+)"/g)].map((match) => match[1]);
const duplicateIds = [...new Set(ids.filter((id, index) => ids.indexOf(id) !== index))];
const referencedIds = scripts.flatMap((script) =>
  [...script.matchAll(/getElementById\("([^"]+)"\)/g)].map((match) => match[1]),
);
const missingReferences = [...new Set(referencedIds.filter((id) => !ids.includes(id)))];
const detailsOpen = (html.match(/<details\b/gi) || []).length;
const detailsClose = (html.match(/<\/details>/gi) || []).length;

const result = {
  syntax: "OK",
  ids: ids.length,
  duplicate_ids: duplicateIds,
  missing_get_element_by_id: missingReferences,
  details_tags: [detailsOpen, detailsClose],
};
console.log(JSON.stringify(result));

if (duplicateIds.length || missingReferences.length || detailsOpen !== detailsClose) process.exit(1);
