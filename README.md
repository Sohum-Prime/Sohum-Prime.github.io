# Sohum Kothavade - Personal Website

Personal website built with Astro, Tailwind CSS, and MDX.

## 🚀 Tech Stack

- **Framework**: [Astro](https://astro.build)
- **Styling**: [Tailwind CSS](https://tailwindcss.com)
- **Content**: MDX with content collections
- **Deployment**: GitHub Pages via GitHub Actions

## 🧞 Commands

| Command                | Action                                           |
| :--------------------- | :----------------------------------------------- |
| `npm install`          | Installs dependencies                            |
| `npm run dev`          | Starts local dev server at `localhost:4321`      |
| `npm run build`        | Build your production site to `./dist/`          |
| `npm run preview`      | Preview your build locally, before deploying     |
| `npm run astro ...`    | Run CLI commands like `astro add`, `astro check` |

## 📁 Project Structure

```
/
├── public/              # Static assets
├── src/
│   ├── components/      # Reusable components
│   ├── content/         # Content collections (blog, projects, publications)
│   ├── layouts/         # Page layouts
│   ├── pages/           # File-based routing
│   └── styles/          # Global styles
├── .github/workflows/   # GitHub Actions
└── package.json
```

## 🌟 Features

- ✅ SEO-friendly with canonical URLs and OpenGraph data
- ✅ RSS feed for blog posts
- ✅ Sitemap generation
- ✅ Responsive design with Tailwind CSS
- ✅ Dark mode support
- ✅ Math rendering with KaTeX
- ✅ Syntax highlighting with Shiki
- ✅ Type-safe content collections
- ✅ Accessible navigation and skip links

## 📝 Content Management

Content is organized into three collections:

- **Blog**: Blog posts in `/src/content/blog/`
- **Projects**: Project showcases in `/src/content/projects/`
- **Publications**: Research publications in `/src/content/publications/`

All content is written in MDX format with YAML frontmatter.

## 📄 License

© 2025 Sohum Kothavade. All rights reserved.
