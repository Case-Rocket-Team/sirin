#![allow(unused_imports)]
use std::collections::HashMap;

use proc_macro::TokenStream;
use quote::ToTokens;
use syn::{parse::{self, Parse}, punctuated::Punctuated, spanned::Spanned, token::Comma, Attribute, DataEnum, DataStruct, Error, Field, Fields, Ident, ItemEnum, ItemStruct, Meta, Token, Variant, Visibility};

use syn::{parse::Parser, parse_macro_input, DeriveInput};
use quote::quote;
use proc_macro2::{Delimiter, Span, TokenStream as TokenStream2, TokenTree};
use anyhow::bail;

// Note, this won't work in downstream crates.
#[proc_macro_derive(SpiError)]
pub fn derive_spi_error(item: TokenStream) -> TokenStream {
    let item: DeriveInput = parse_macro_input!(item);
    let name = item.ident;
    let (impl_generics, ty_generics, where_clause) = item.generics.split_for_impl();

    quote! {
        impl #impl_generics embedded_hal_async::spi::ErrorType for #name #ty_generics #where_clause {
            type Error = crate::spi::SpiError;
        }
    }.into()
}

#[proc_macro_derive(Measurement)]
pub fn derive_measurement(item: TokenStream) -> TokenStream {
    let item: DeriveInput = parse_macro_input!(item);
    let name = item.ident;

    let syn::Data::Struct(data) = item.data else {
        return quote! {compile_error!("Only structs allowed")}.into()
    };

    let fields = data.fields.iter().map(|f| f.ident.clone());

    let (impl_generics, ty_generics, where_clause) = item.generics.split_for_impl();

    quote! {
        impl #impl_generics crate::subsystems::Measurement for #name #ty_generics #where_clause {
            fn unmeasured() -> Self {
                Self {
                    #(#fields: Err(crate::subsystems::SubsystemError::NotYetMeasured)),*
                }
            }
        }
    }.into()
}

// TODO: maybe do something about this code duplication...

#[proc_macro_derive(SongSize, attributes(song))]
pub fn derive_song_size(tok: TokenStream) -> TokenStream {
    let tok1 = tok.clone();
    let item: DeriveInput = parse_macro_input!(tok1);

    match item.data {
        syn::Data::Struct(ref s) => derive_song_size_struct(&item, s).unwrap().into(),
        syn::Data::Enum(_) => {
            let enum_song: EnumSong = parse_macro_input!(tok);
            derive_song_size_enum(&enum_song).unwrap().into()
        },
        _ => todo!()
    }
}

#[proc_macro_derive(ToSong, attributes(song))]
pub fn derive_to_song(tok: TokenStream) -> TokenStream {
    let tok1 = tok.clone();
    let item: DeriveInput = parse_macro_input!(tok1);

    match item.data {
        syn::Data::Struct(ref s) => derive_to_song_struct(&item, &s).unwrap().into(),
        syn::Data::Enum(_) => {
            let enum_song: EnumSong = parse_macro_input!(tok);
            derive_to_song_enum(&enum_song).unwrap().into()
        },
        _ => todo!()
    }
}

#[proc_macro_derive(FromSong, attributes(song))]
pub fn derive_from_song(tok: TokenStream) -> TokenStream {
    let tok1 = tok.clone();
    let item: DeriveInput = parse_macro_input!(tok1);

    match item.data {
        syn::Data::Struct(ref s) => derive_from_song_struct(&item, s).unwrap().into(),
        syn::Data::Enum(_) => {
            let enum_song: EnumSong = parse_macro_input!(tok);
            derive_from_song_enum(&enum_song).unwrap().into()
        },
        _ => todo!()
    }
}

enum EnumSongDisc {
    Enum {
        disc_name: Ident,
        disc_type: Ident,
    },
    Repr {
        ty: Ident
    }
}

struct EnumSong {
    item: ItemEnum,
    disc: EnumSongDisc
}

impl Parse for EnumSong {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        let item: ItemEnum = input.parse()?;

        let attrs: Vec<TokenStream2> = item.attrs.iter().filter_map(|a| {
            if a.path().get_ident()?.to_string() != "song" {
                return None;
            }
    
            let Meta::List(ref list) = a.meta else {
                return None
            };
    
            Some(list.tokens.clone())
        }).collect();

        let repr = item.attrs.iter()
            .filter(|attr| attr.path().to_token_stream().to_string() == "repr")
            .filter_map(|attr| attr.parse_args::<Ident>().ok())
            .next();

        if let Some(repr) = repr {
            if attrs.len() != 0 {
                return Err(Error::new(
                    attrs.get(0).map(|i| i.span()).unwrap_or_else(|| item.span()),
                    "Expected no attrs"
                ));                
            }

            return Ok(EnumSong {
                item,
                disc: EnumSongDisc::Repr {
                    ty: repr
                }
            })
        }

        if attrs.len() != 1 {
            return Err(Error::new(
                attrs.get(0).map(|i| i.span()).unwrap_or_else(|| item.span()),
                "Expected 1 attr -- discriminant"
            ));
        }

        let mut attr = attrs[0].clone().into_iter();

        let disc_ident: Ident = syn::parse2(attr.next().unwrap().to_token_stream())?;
        if disc_ident.to_string() != "discriminant" {
            return Err(Error::new_spanned(disc_ident, "Expected 'discriminant'"));
        }

        let group: proc_macro2::Group = syn::parse2(attr.next().unwrap().to_token_stream())?;
        if group.delimiter() != Delimiter::Parenthesis {
            return Err(Error::new(group.delim_span().span(), "Expected parens"));
        }

        let mut iter = group.stream().into_token_stream().into_iter();
        let disc_name: Ident = syn::parse2(iter.next().unwrap().into_token_stream())?;
        let _: Token![=] = syn::parse2(iter.next().unwrap().into_token_stream())?;
        let disc_type: Ident = syn::parse2(iter.next().unwrap().into_token_stream())?;

        /*if disc_type.to_string() != "u8" {
            return Err(Error::new(disc_type.span(), "Only u8 supported rn"));
        }*/

        Ok(EnumSong {
            item,
            disc: EnumSongDisc::Enum {
                disc_name,
                disc_type,
            }
        })
    }
}

fn derive_song_size_struct(item: &DeriveInput, data: &DataStruct) -> Result<TokenStream2, anyhow::Error> {
    let mut size_out = vec![];

    let mut has_song_size = quote! {()};

    for field in &data.fields {
        let Field { ident, ty, .. } = field;

        size_out.push(quote! {
            self.#ident.song_size()
        });

        has_song_size = quote! { (#has_song_size, ConstSongSizeImplFromConstSongSize<#ty>) }
    }

    let ident = &item.ident;
    let (impl_generics, ty_generics, where_clause) = item.generics.split_for_impl();
    
    Ok(quote! {
        #[automatically_derived]
        impl #impl_generics HasSongSize for #ident #ty_generics #where_clause {
            type Size = #has_song_size;
        }

        #[automatically_derived]
        impl #impl_generics SongSize for #ident #ty_generics #where_clause {
            fn song_size(&self) -> usize {
                0 #( + #size_out)*
            }
        }
    })
}

fn derive_to_song_struct(item: &DeriveInput, data: &DataStruct) -> Result<TokenStream2, anyhow::Error> {
    let mut fields_out = vec![];

    for field in &data.fields {
        let ident = &field.ident;

        fields_out.push(quote! {
            self.#ident.to_song(&mut buf[i..])?;
            i += self.#ident.song_size();
        });
    }

    let ident = &item.ident;
    let (impl_generics, ty_generics, where_clause) = item.generics.split_for_impl();

    Ok(quote! {
        impl #impl_generics ToSong for #ident #ty_generics #where_clause {
            fn to_song(&self, buf: &mut [u8]) -> Result<(), ToSongError> {
                let size = self.song_size();
                if buf.len() >= size {
                    let mut i = 0;
                    #(#fields_out)*
                    Ok(())
                } else {
                    Err(ToSongError::BufferOverflow)
                }
            }
        }
    })
}

fn derive_from_song_struct(item: &DeriveInput, data: &DataStruct) -> Result<TokenStream2, anyhow::Error> {
    let mut from_song_out = vec![];
    
    for field in &data.fields {
        let Field { ident, ty, .. } = field;

        from_song_out.push(quote! {
            #ident: {
                let value = <#ty as FromSong>::from_song(&buf[i..])?;
                i += value.song_size();
                value
            }
        });
    }

    let ident = &item.ident;
    let (impl_generics, ty_generics, where_clause) = item.generics.split_for_impl();

    Ok(quote! {
        impl #impl_generics FromSong for #ident #ty_generics #where_clause {
            fn from_song(buf: &[u8]) -> Result<Self, FromSongError> {
                let mut i = 0;
                Ok(
                    #ident {
                        #(#from_song_out,)*
                    }
                )
            }
        }
    })
}

fn derive_song_size_enum(enum_song: &EnumSong) -> syn::Result<TokenStream2> {
    let ident = &enum_song.item.ident;
    
    match &enum_song.disc {
        EnumSongDisc::Repr { .. } => {
            Ok(quote! {
                #[automatically_derived]
                impl HasSongSize for #ident {
                    type Size = ConstSongSizeValue<{ core::mem::size_of::<Self>() }>;
                }

                #[automatically_derived]
                impl SongSize for #ident {
                    fn song_size(&self) -> usize {
                        core::mem::size_of::<Self>()
                    }
                }
            })
        },
        EnumSongDisc::Enum { disc_name, disc_type } => {
            let mut out = vec![];
            let mut disc_out = vec![];
            let mut song_disc_out = vec![];

            for var in &enum_song.item.variants {
                let disc_var_ident = &var.ident;

                match &var.discriminant {
                    Some((eq, val)) => {
                        disc_out.push(quote! {
                            #disc_var_ident #eq #val
                        });
                    }
                    None => {
                        disc_out.push(quote! {
                            #disc_var_ident
                        });
                    }
                }

                let mut idents = vec![];

                let mut i = 0;
                for field in &var.fields {
                    idents.push(
                        field.ident.clone().unwrap_or_else(
                            || Ident::new(&format!("t{}", i).to_string(), field.span())
                        )
                    );
                    i += 1;
                }

                let mut fields_out = vec![];

                for ident in &idents {
                    fields_out.push(quote! {
                        i += #ident.song_size();
                    });
                }

                let ident = &var.ident;

                let destructure = match &var.fields {
                    Fields::Unit => quote!(),
                    Fields::Named(_) => quote!({ #(#idents),* }),
                    Fields::Unnamed(_) => quote!(( #(#idents),* ))
                };

                out.push(quote! {
                    Self::#ident #destructure => {
                        let mut i = core::mem::size_of::<#disc_type>();
                        #(#fields_out)*
                        i
                    }
                });

                song_disc_out.push(quote! {
                    Self::#ident #destructure => #disc_name::#disc_var_ident
                });
            }

            let vis = &enum_song.item.vis;

            Ok(quote! {
                #[derive(Clone, Copy, PartialEq, Eq, Debug, SongSize, ToSong, FromSong)]
                #[repr(#disc_type)]
                #vis enum #disc_name {
                    #(#disc_out),*
                }

                #[automatically_derived]
                impl SongDiscriminant for #ident {
                    type Discriminant = #disc_name;

                    fn song_discriminant(&self) -> Self::Discriminant {
                        match self {
                            #(#song_disc_out),*
                        }
                    }
                }

                #[automatically_derived]
                impl SongSize for #ident {
                    fn song_size(&self) -> usize {
                        match self {
                            #(#out),*
                        }
                    }
                }
            })
        }
    }
}

fn derive_to_song_enum(enum_song: &EnumSong) -> syn::Result<TokenStream2> {
    let ident = &enum_song.item.ident;

    match &enum_song.disc {
        EnumSongDisc::Repr { ty } => {
            Ok(quote! {
                #[automatically_derived]
                impl ToSong for #ident {
                    fn to_song(&self, buf: &mut [u8]) -> Result<(), ToSongError> {
                        (*self as #ty).to_song(buf)
                    }
                }
            })
        },
        EnumSongDisc::Enum { disc_name, disc_type } => {
            let mut out = vec![];

            for var in &enum_song.item.variants {
                let mut idents = vec![];

                let mut i = 0;
                for field in &var.fields {
                    // println!("Debug 100: {:?}", field);
                    idents.push(
                        field.ident.clone().unwrap_or_else(
                            || Ident::new(&format!("t{}", i).to_string(), field.span())
                        )
                    );
                    i += 1;
                }

                let mut fields_out = vec![];

                for ident in &idents {
                    fields_out.push(quote! {
                        #ident.to_song(&mut buf[i..])?;
                        i += #ident.song_size();
                    });
                }

                let ident = &var.ident;

                let destructure = match &var.fields {
                    Fields::Unit => quote!(),
                    Fields::Named(_) => quote!({ #(#idents),* }),
                    Fields::Unnamed(_) => quote!(( #(#idents),* ))
                };

                out.push(quote! {
                    Self::#ident #destructure => {
                        (#disc_name::#ident as #disc_type).to_song(buf)?;
                        let mut i = core::mem::size_of::<#disc_type>();
                        #(#fields_out)*
                        Ok(())
                    }
                });
            }
            
            Ok(quote! {
                impl ToSong for #ident {
                    fn to_song(&self, buf: &mut [u8]) -> Result<(), ToSongError> {
                        match self {
                            #(#out),*
                        }
                    }
                }
            })
        }
    }
}

fn derive_from_song_enum(enum_song: &EnumSong) -> syn::Result<TokenStream2> {
    let ident = &enum_song.item.ident;
    
    match &enum_song.disc {
        EnumSongDisc::Repr { ty } => {
            let var = enum_song.item.variants.iter().map(|var| &var.ident);

            Ok(quote! {
                impl FromSong for #ident {
                    fn from_song(buf: &[u8]) -> Result<Self, FromSongError> {
                        if buf.len() < core::mem::size_of::<#ty>() {
                            return Err(FromSongError::BufferOverflow);
                        }

                        match buf[0] {
                            #(disc if disc == #ident::#var as #ty => Ok(#ident::#var),)*
                            _ => Err(FromSongError::InvalidPacketId)
                        }
                    }
                }
            })
        }
        EnumSongDisc::Enum { disc_name, disc_type } => {
            let mut out = vec![];
            for var in &enum_song.item.variants {

                let mut fields_out = vec![];
                let mut idents = vec![];
                
                let mut i = 0;
                for field in &var.fields {
                    let ident = field.ident.clone().unwrap_or_else(
                        || Ident::new(&format!("t{}", i).to_string(), field.span()));
                    idents.push(ident.clone());
                    
                    let typ = field.ty.clone();
                    
                    fields_out.push(quote! {
                        let #ident = {
                            // avoid turbofish problem
                            type Ty = #typ;
                            Ty::from_song(&buf[i..])?
                        };

                        i += #ident.song_size();
                    });
                    i += 1;
                }   

                let var_ident = &var.ident;


                match &var.fields {
                    Fields::Unit => out.push(quote! {
                            val if val == #disc_name::#var_ident as #disc_type => Ok(#ident::#var_ident)
                        }),
                    Fields::Unnamed(_) => out.push(quote! { 
                        val if val == #disc_name::#var_ident as #disc_type => {
                            let mut i = 1;
                            #(#fields_out)*
                            Ok(#ident::#var_ident(#(#idents,)*))
                        }}),
                    Fields::Named(_) => todo!()
                }
            }
            
            Ok(quote! {
                impl FromSong for #ident {
                    fn from_song(buf: &[u8]) -> Result<Self, FromSongError> {
                        let Some(&disc) = buf.get(0) else {
                            return Err(FromSongError::BufferOverflow)
                        };

                        match disc {
                            #(#out,)*
                            _ => Err(FromSongError::InvalidPacketId)
                        }
                    }
                }
            })
        }
    }
}