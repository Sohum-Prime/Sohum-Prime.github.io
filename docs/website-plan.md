# Sohum Kothavade - Personal Website Plan

## Overview
Building a stunning, performant, and professional personal website for an AI researcher with interests in robotics, math/physics, and related fields. The site will showcase research, publications, projects, and blog content.

## Stack Selection

### Core Framework: Astro
**Why Astro:**
- **Best-in-class performance**: Ships zero JavaScript by default, partial hydration for interactive components
- **Perfect for content-heavy sites**: Built-in content collections, MDX support
- **SEO-first**: Server-side rendering, automatic sitemap/RSS generation
- **Framework agnostic**: Can integrate React, Vue, Svelte if needed
- **Developer experience**: Fast builds, HMR, TypeScript support
- **GitHub Pages ready**: Static site generation with easy deployment

### Styling: Tailwind CSS
- Utility-first CSS with modern design capabilities
- Excellent performance (purges unused styles)
- Responsive design system
- Easy dark mode implementation
- Great for academic/professional aesthetics

### Content: MDX + Content Collections
- Write content in Markdown with embedded components
- Type-safe content management via Astro Content Collections
- LaTeX math support via remark-math/rehype-katex
- Syntax highlighting for code via Shiki
- Bibliography/citations support

### Additional Features
- **Search**: Pagefind for client-side search
- **Comments**: Giscus (GitHub Discussions-based)
- **Images**: Astro's built-in image optimization
- **RSS/Sitemap**: Auto-generated for blog
- **Analytics**: Can add Plausible/Google Analytics later

## Site Architecture

### Information Architecture
```
/                     → Home (About/Landing)
/publications         → Publications list with filtering
/projects            → Projects showcase
/blog                → Blog index
/blog/[slug]         → Individual blog posts
/404                 → Custom 404 page
```

### Content Model

#### Publications
```typescript
{
  title: string
  authors: string[]
  venue: string
  year: number
  abstract: string
  pdf?: string
  arxiv?: string
  github?: string
  slides?: string
  bibtex: string
  tags: string[]
}
```

#### Projects
```typescript
{
  title: string
  description: string
  thumbnail: string
  tags: string[]
  github?: string
  demo?: string
  status: 'active' | 'completed' | 'archived'
  featured: boolean
}
```

#### Blog Posts
```typescript
{
  title: string
  description: string
  publishDate: Date
  updatedDate?: Date
  tags: string[]
  draft: boolean
  featured: boolean
}
```

## Design Principles

### Visual Design
- **Clean & Professional**: Minimalist design focusing on content
- **Typography-first**: Strong typographic hierarchy
- **Academic aesthetic**: Inspired by research-focused sites (distill.pub, colah.github.io)
- **Accessible**: WCAG AA compliance, semantic HTML
- **Performance**: < 50KB initial page load, < 1s Time to Interactive

### Color Scheme
- Light mode: Clean whites, subtle grays, accent color (blue/teal for links)
- Dark mode: True dark background (not gray), high contrast text
- Syntax highlighting: GitHub-style for light, One Dark Pro for dark

### Typography
- Headings: Inter or similar geometric sans-serif
- Body: System font stack or Literata for readability
- Code: JetBrains Mono or Fira Code

## Development Milestones

### Phase 1: Cleanup & Setup (Current)
- [x] Remove existing files
- [x] Create temporary placeholder
- [x] Create documentation structure
- [ ] Merge cleanup PR

### Phase 2: Astro Scaffold
- [ ] Initialize Astro project
- [ ] Set up Tailwind CSS
- [ ] Configure TypeScript
- [ ] Set up content collections (blog, projects, publications)
- [ ] Create base layout and navigation
- [ ] Configure image optimization
- [ ] Set up RSS/Sitemap
- [ ] Add math/code syntax support

### Phase 3: MVP Pages
- [ ] Home/About page with hero section
- [ ] Publications page with filtering
- [ ] Projects page with grid layout
- [ ] Blog index page
- [ ] Individual blog post template
- [ ] 404 page

### Phase 4: Deployment & CI/CD
- [ ] GitHub Actions workflow for build
- [ ] GitHub Pages deployment
- [ ] Custom domain setup (if applicable)
- [ ] Performance testing
- [ ] Accessibility audit

### Phase 5: Content & Polish
- [ ] Add actual publications data
- [ ] Add project entries
- [ ] Write initial blog posts
- [ ] Add search functionality (Pagefind)
- [ ] Add comments (Giscus)
- [ ] SEO optimization (meta tags, OG images)

## GitHub Pages Deployment

### Configuration
- **Build method**: GitHub Actions
- **Source**: `gh-pages` branch or `/docs` folder
- **Custom domain**: Optional (configure DNS if needed)

### GitHub Actions Workflow
```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 20
      - run: npm ci
      - run: npm run build
      - uses: actions/upload-pages-artifact@v3
        with:
          path: ./dist
  
  deploy:
    needs: build
    runs-on: ubuntu-latest
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    steps:
      - uses: actions/deploy-pages@v4
        id: deployment
```

## Performance Targets
- **Lighthouse Score**: 95+ across all metrics
- **Initial Load**: < 50KB (gzipped)
- **Time to Interactive**: < 1s
- **First Contentful Paint**: < 0.5s
- **Build Time**: < 1min for full site

## Accessibility Requirements
- WCAG 2.1 AA compliance
- Semantic HTML5
- Keyboard navigation
- Screen reader tested
- Color contrast ratios
- Focus indicators
- Skip links

## SEO Optimization
- Semantic HTML structure
- Meta tags (title, description, OG, Twitter)
- XML sitemap
- RSS feed
- Structured data (JSON-LD)
- Fast load times
- Mobile responsive

## Next Steps
1. Merge cleanup PR
2. Initialize Astro scaffold on new branch
3. Set up base layout and navigation
4. Create content collections
5. Build MVP pages
6. Set up GitHub Actions
7. Deploy to GitHub Pages
8. Add content and iterate

---

*Last updated: October 31, 2025*
