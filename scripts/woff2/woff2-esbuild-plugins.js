const fs = require("fs");
const path = require("path");
const fetch = require("node-fetch");

const wawoff = require("wawoff2");
const { Font } = require("fonteditor-core");

/**
 * Custom esbuild plugin to convert url woff2 imports into a text.
 * Other woff2 imports are handled by a "file" loader.
 *
 * @returns {import("esbuild").Plugin}
 */
module.exports.woff2BrowserPlugin = () => {
  return {
    name: "woff2BrowserPlugin",
    setup(build) {
      build.initialOptions.loader = {
        ".woff2": "file",
        ...build.initialOptions.loader,
      };

      build.onResolve({ filter: /^https:\/\/.+?\.woff2$/ }, (args) => {
        return {
          path: args.path,
          namespace: "woff2BrowserPlugin",
        };
      });

      build.onLoad(
        { filter: /.*/, namespace: "woff2BrowserPlugin" },
        async (args) => {
          return {
            contents: args.path,
            loader: "text",
          };
        },
      );
    },
  };
};

/**
 * Custom esbuild plugin to:
 * 1. inline all woff2 (url and relative imports) as base64 for server-side use cases (no need for additional font fetch; works in both esm and commonjs)
 * 2. convert all the imported fonts (including those from cdn) at build time into .ttf (since Resvg does not support woff2, neither inlined dataurls - https://github.com/RazrFalcon/resvg/issues/541)
 *    - merging multiple woff2 into one ttf (for same families with different unicode ranges)
 *    - deduplicating glyphs due to the merge process
 *    - merging emoji font for each
 *    - printing out font metrics
 *
 * @returns {import("esbuild").Plugin}
 */
module.exports.woff2ServerPlugin = (options = {}) => {
  // google CDN fails time to time, so let's retry
  async function fetchRetry(url, options = {}, retries = 0, delay = 1000) {
    try {
      const response = await fetch(url, options);

      if (!response.ok) {
        throw new Error(`Status: ${response.status}, ${await response.json()}`);
      }

      return response;
    } catch (e) {
      if (retries > 0) {
        await new Promise((resolve) => setTimeout(resolve, delay));
        return fetchRetry(url, options, retries - 1, delay * 2);
      }

      console.error(`Couldn't fetch: ${url}, error: ${e.message}`);
      throw e;
    }
  }

  const notoEmojiBuffer = fs.readFileSync(
    path.resolve(__dirname, "./assets/NotoEmoji-Regular.ttf"),
  );

  return {
    name: "woff2ServerPlugin",
    setup(build) {
      const { outdir, generateTtf } = options;
      const outputDir = path.resolve(outdir);
      const fonts = new Map();

      build.onResolve({ filter: /\.woff2$/ }, (args) => {
        const resolvedPath = args.path.startsWith("http")
          ? args.path // url
          : path.resolve(args.resolveDir, args.path); // absolute path

        return {
          path: resolvedPath,
          namespace: "woff2ServerPlugin",
        };
      });

      build.onLoad(
        { filter: /.*/, namespace: "woff2ServerPlugin" },
        async (args) => {
          let woff2Buffer;

          if (path.isAbsolute(args.path)) {
            // read local woff2 as a buffer (WARN: `readFileSync` does not work!)
            woff2Buffer = await fs.promises.readFile(args.path);
          } else {
            // fetch remote woff2 as a buffer (i.e. from a cdn)
            const response = await fetchRetry(args.path, {}, 3);
            woff2Buffer = await response.buffer();
          }

          // google's brotli decompression into ttf
          const snftBuffer = new Uint8Array(
            await wawoff.decompress(woff2Buffer),
          ).buffer;

          // load font and store per fontfamily & subfamily cache
          let font;

          try {
            font = Font.create(snftBuffer, { type: "ttf" });
          } catch {
            // if loading as ttf fails, try to load as otf
            font = Font.create(snftBuffer, { type: "otf" });
          }

          const fontFamily = font.data.name.fontFamily;
          const subFamily = font.data.name.fontSubFamily;

          if (!fonts.get(fontFamily)) {
            fonts.set(fontFamily, {});
          }

          if (!fonts.get(fontFamily)[subFamily]) {
            fonts.get(fontFamily)[subFamily] = [];
          }

          // store the snftbuffer per subfamily
          fonts.get(fontFamily)[subFamily].push(font);

          // inline the woff2 as base64 for server-side use cases
          // NOTE: "file" loader is broken in commonjs and "dataurl" loader does not produce correct ur
          return {
            contents: `data:font/woff2;base64,${woff2Buffer.toString(
              "base64",
            )}`,
            loader: "text",
          };
        },
      );

      // TODO: strip away some unnecessary glyphs
      build.onEnd(() => {
        if (!generateTtf) {
          return;
        }

        const sortedFonts = Array.from(fonts.entries()).sort(
          ([family1], [family2]) => (family1 > family2 ? 1 : -1),
        );

        // for now we are interested in the regular families only
        for (const [family, { Regular }] of sortedFonts) {
          // merge same previous woff2 subfamilies into one font and sort the glpyhs
          const [head, ...tail] = Regular;
          const mergedFont = tail
            .reduce((acc, curr) => {
              return acc.merge(curr);
            }, head)
            .sort();

          // merge with emoji font
          const font = mergedFont.merge(
            Font.create(notoEmojiBuffer, { type: "ttf" }),
          );

          // deduplicate glyphs by name+unicode due to merge
          const uniqueGlyphs = new Set();
          const glyphs = [...font.data.glyf].filter((x) => {
            if (!x.unicode) {
              return true;
            }

            if (!uniqueGlyphs.has(x.unicode.toString())) {
              uniqueGlyphs.add(x.unicode.toString());
              return true;
            }

            return false;
          });

          // deduplucate ".notdef" glyph as it's unicodes are not cleaned after merge
          const notDefGlyph = glyphs.find((x) => x.name === ".notdef");
          if (notDefGlyph && Array.isArray(notDefGlyph.unicode)) {
            notDefGlyph.unicode = notDefGlyph.unicode.filter(
              (x) => !uniqueGlyphs.has(x.toString()),
            );
          }

          const duplicateGlyphssLength = font.data.glyf.length - glyphs.length;

          font.set({
            ...font.data,
            glyf: glyphs,
          });

          const extension = "ttf";
          const fileName = `${family}.${extension}`;
          const { ascent, descent } = font.data.hhea;

          if (!fs.existsSync(outputDir)) {
            fs.mkdirSync(outputDir, { recursive: true });
          }

          // write down the buffer
          fs.writeFileSync(
            path.resolve(outputDir, fileName),
            font.write({ type: extension }),
          );

          console.info(`Generated "${fileName}"`);
          if (Regular.length > 1) {
            console.info(`- by merging ${Regular.length} woff2 files`);
          }
          if (duplicateGlyphssLength) {
            console.info(`- deduplicated ${duplicateGlyphssLength} glyphs`);
          }
          console.info(
            `- with metrics ${font.data.head.unitsPerEm}, ${ascent}, ${descent}`,
          );
          console.info(``);
        }
      });
    },
  };
};
